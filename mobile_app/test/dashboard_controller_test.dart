import 'dart:async';
import 'dart:convert';

import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:motioncore_mobile/controllers/dashboard_controller.dart';
import 'package:motioncore_mobile/services/robot_api.dart';

final statusData = <String, dynamic>{
  'status': 'IDLE',
  'motion': 'STAND',
  'speed': 0.2,
  'controller': 'NONE',
  'connected': true,
};
final telemetryData = <String, dynamic>{
  'height': 0.773,
  'roll': -0.009,
  'pitch': -0.059,
  'leftFoot': false,
  'rightFoot': true,
  'fallen': false,
};
http.Response jsonResponse(Object data) => http.Response(jsonEncode(data), 200);

void main() {
  test(
    'polling preserves the operator speed and updates actual telemetry',
    () async {
      final model = DashboardController(
        api: RobotApi(
          client: MockClient(
            (request) async => jsonResponse(
              request.url.path.endsWith('status') ? statusData : telemetryData,
            ),
          ),
        ),
      );
      addTearDown(model.dispose);
      model.setSpeed(0.8);
      await model.refresh();
      expect(model.requestedSpeed, 0.8);
      expect(model.status!.speed, 0.2);
      expect(model.telemetry!.height, 0.773);
      expect(model.telemetry!.leftFoot, isFalse);
      expect(model.connected, isTrue);
    },
  );

  test('telemetry failure does not hide a successful status response; recovery works', () async {
    var fail = true;
    final model = DashboardController(
      api: RobotApi(
        client: MockClient((request) async {
          if (request.url.path.endsWith('status')) {
            return jsonResponse(statusData);
          }
          return fail ? http.Response('bad', 500) : jsonResponse(telemetryData);
        }),
      ),
    );
    addTearDown(model.dispose);
    await model.refresh();
    expect(model.connected, isTrue);
    expect(model.telemetryFresh, isFalse);
    expect(model.telemetryError, isNotNull);
    fail = false;
    await model.refresh();
    expect(model.telemetryFresh, isTrue);
    expect(model.telemetryError, isNull);
  });

  test(
    'failed or old status never remains connected and STOP remains available',
    () async {
      var time = DateTime(2026, 9, 7);
      var fail = false;
      var stops = 0;
      final model = DashboardController(
        now: () => time,
        api: RobotApi(
          client: MockClient((request) async {
            if (request.url.path.endsWith('stop')) {
              stops++;
              return jsonResponse({'success': true});
            }
            if (request.url.path.endsWith('status')) {
              return fail ? http.Response('', 503) : jsonResponse(statusData);
            }
            return jsonResponse(telemetryData);
          }),
        ),
      );
      addTearDown(model.dispose);
      await model.refresh();
      time = time.add(const Duration(seconds: 7));
      expect(model.connected, isFalse);
      expect(model.telemetryFresh, isFalse);
      fail = true;
      await model.refresh();
      expect(model.statusError, isNotNull);
      await model.stop();
      expect(stops, 1);
    },
  );

  test('poll requests do not overlap', () async {
    final pending = Completer<http.Response>();
    var calls = 0;
    final model = DashboardController(
      api: RobotApi(
        client: MockClient((request) {
          calls++;
          return request.url.path.endsWith('status')
              ? pending.future
              : Future.value(jsonResponse(telemetryData));
        }),
      ),
    );
    addTearDown(model.dispose);
    final first = model.refresh();
    final second = model.refresh();
    pending.complete(jsonResponse(statusData));
    await Future.wait([first, second]);
    expect(calls, 2);
  });

  test(
    'duplicate WALK is blocked and STOP is sent after the pending action',
    () async {
      final pending = Completer<http.Response>();
      final commands = <String>[];
      final model = DashboardController(
        api: RobotApi(
          client: MockClient((request) async {
            if (request.method == 'POST') {
              commands.add(request.url.path);
              if (request.url.path.endsWith('action')) return pending.future;
              return jsonResponse({
                'success': true,
                'message': 'RL controller stopped',
              });
            }
            return jsonResponse(
              request.url.path.endsWith('status') ? statusData : telemetryData,
            );
          }),
        ),
      );
      addTearDown(model.dispose);
      await model.refresh();
      final walk = model.sendAction('WALK');
      await model.sendAction('WALK');
      final stop = model.stop();
      expect(model.stopping, isTrue);
      pending.complete(
        jsonResponse({'success': true, 'message': 'RL controller launched'}),
      );
      await Future.wait([walk, stop]);
      expect(commands, ['/robot/action', '/robot/stop']);
      expect(model.commandMessage, 'RL controller stopped');
    },
  );

  test('old server replies cannot replace new connection data', () async {
    final oldStatus = Completer<http.Response>();
    final model = DashboardController(
      api: RobotApi(
        client: MockClient((request) async {
          if (request.url.path.endsWith('status')) {
            return request.url.host == '10.200.233.20'
                ? oldStatus.future
                : jsonResponse({...statusData, 'status': 'STOPPED'});
          }
          return jsonResponse(telemetryData);
        }),
      ),
    );
    addTearDown(model.dispose);
    final refresh = model.refresh();
    final change = model.setServer('http://192.168.1.2:8000');
    oldStatus.complete(jsonResponse({...statusData, 'status': 'WALKING'}));
    await Future.wait([refresh, change]);
    expect(model.status!.status, 'STOPPED');
  });

  test('dispose while polling does not notify disposed listeners', () async {
    final pending = Completer<http.Response>();
    final model = DashboardController(
      api: RobotApi(client: MockClient((_) => pending.future)),
    );
    final refresh = model.refresh();
    model.dispose();
    pending.complete(jsonResponse(statusData));
    await refresh;
  });
  test('speed changes are sent before a following stop', () async {
    final pending = Completer<http.Response>();
    final commands = <String>[];
    final model = DashboardController(api: RobotApi(client: MockClient((request) async {
      if (request.method == 'POST') {
        commands.add(request.url.path);
        if (request.url.path == '/robot/speed') return pending.future;
        return jsonResponse({'success': true, 'message': 'Stopping'});
      }
      return jsonResponse(request.url.path.endsWith('status')
          ? {...statusData, 'status': 'WALKING'} : telemetryData);
    })));
    addTearDown(model.dispose);
    await model.refresh();
    model.setSpeed(.7);
    final speed = model.applySpeed();
    final stop = model.stop();
    pending.complete(jsonResponse({'success': true, 'message': 'Speed sent'}));
    await Future.wait([speed, stop]);
    expect(commands, ['/robot/speed', '/robot/stop']);
    expect(model.commandMessage, 'Stopping');
  });

  test('starting RL prevents duplicate walk and gesture commands', () async {
    final model = DashboardController(api: RobotApi(client: MockClient((request) async {
      return jsonResponse(request.url.path.endsWith('status')
          ? {...statusData, 'status': 'STARTING'} : telemetryData);
    })));
    addTearDown(model.dispose);
    await model.refresh();
    expect(model.canWalk, isFalse);
    expect(model.canDemoAction, isFalse);
  });

}
