import 'dart:async';
import 'dart:convert';

import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:motioncore_mobile/services/robot_api.dart';
import 'package:motioncore_mobile/models/robot_telemetry.dart';

void main() {
  test('uses the existing WALK payload and dedicated stop endpoint', () async {
    final requests = <http.Request>[];
    final api = RobotApi(
      client: MockClient((request) async {
        requests.add(request);
        return http.Response('{"success":true}', 200);
      }),
    );
    addTearDown(api.close);
    await api.sendCommand('WALK', 0.65);
    await api.stopRobot();
    expect(requests[0].url.path, '/robot/action');
    expect(requests[0].method, 'POST');
    expect(jsonDecode(requests[0].body), {'action': 'WALK', 'speed': 0.65});
    expect(requests[1].url.path, '/robot/stop');
    expect(requests[1].method, 'POST');
  });

  for (final response in [
    http.Response('{"success":false,"message":"Robot already walking"}', 200),
    http.Response('{"detail":"failed"}', 500),
    http.Response('not json', 200),
    http.Response('[]', 200),
    http.Response('{}', 200),
  ]) {
    test('rejects unconfirmed command: ${response.body}', () async {
      final api = RobotApi(client: MockClient((_) async => response));
      addTearDown(api.close);
      await expectLater(
        api.sendCommand('WALK', 0.5),
        throwsA(isA<RobotApiException>()),
      );
    });
  }

  test('network requests have a finite timeout', () async {
    final pending = Completer<http.Response>();
    final api = RobotApi(
      timeout: const Duration(milliseconds: 5),
      client: MockClient((_) => pending.future),
    );
    addTearDown(api.close);
    await expectLater(api.getRobotStatus(), throwsA(isA<RobotApiException>()));
    pending.complete(http.Response('{}', 200));
  });

  test('server settings accept an origin and reject paths or credentials', () {
    expect(
      RobotApi.normalizeBaseUrl(' http://192.168.1.2:8000/ '),
      'http://192.168.1.2:8000',
    );
    for (final url in [
      'localhost:8000',
      'ftp://host',
      'http://host/api',
      'http://user:pass@host',
      'http://host?x=1',
      'http://host:99999',
    ]) {
      expect(() => RobotApi.normalizeBaseUrl(url), throwsFormatException);
    }
  });

  test('missing and fallback telemetry cannot be shown as stable', () {
    expect(() => RobotTelemetry.fromJson({}), throwsFormatException);
    expect(
      () => RobotTelemetry.fromJson({
        'height': 0,
        'roll': 0,
        'pitch': 0,
        'leftFoot': false,
        'rightFoot': false,
        'fallen': false,
      }),
      throwsFormatException,
    );
  });
}
