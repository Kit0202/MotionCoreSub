import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/testing.dart';
import 'package:motioncore_mobile/controllers/dashboard_controller.dart';
import 'package:motioncore_mobile/main.dart';
import 'package:motioncore_mobile/models/robot_status.dart';
import 'package:motioncore_mobile/screens/home_page.dart';
import 'package:motioncore_mobile/services/robot_api.dart';

import 'dashboard_controller_test.dart' show jsonResponse, telemetryData;

Map<String, dynamic> pickupStatus(String state, List<String> actions) => {
  'scene': 'pickup', 'status': state, 'motion': state, 'speed': 0.0,
  'connected': true, 'controller': 'CAN_PICKUP',
  'availableActions': actions, 'holding': state == 'HOLDING',
  'progress': state == 'GRABBING' ? .2 : null,
  'message': 'Can pickup status',
  'selectedCan': 'cola',
  'canTypes': [
    {'id':'cola', 'label':'Cola can'},
    {'id':'tuna', 'label':'Tuna can'},
    {'id':'100plus', 'label':'100PLUS can'},
    {'id':'pepsi', 'label':'Pepsi can'},
  ],
  'canSelectionAllowed': state == 'READY',
};

void main() {
  test('pickup permissions follow the backend and actions use the existing API', () async {
    var status = pickupStatus('READY', ['GRAB', 'RESET', 'STOP']);
    final commands = <String>[];
    final model = DashboardController(api: RobotApi(client: MockClient((request) async {
      if (request.method == 'POST') {
        final action = (jsonDecode(request.body) as Map)['action'] as String;
        commands.add(action);
        status = pickupStatus('GRABBING', ['STOP']);
        return jsonResponse({'success': true, 'message': 'Reaching'});
      }
      return jsonResponse(request.url.path.endsWith('status') ? status : telemetryData);
    })));
    addTearDown(model.dispose);
    await model.refresh();
    expect(model.isPickupScene, isTrue);
    expect(model.canWalk, isFalse);
    expect(model.canPickupAction('GRAB'), isTrue);
    expect(model.canPickupAction('PLACE'), isFalse);
    await model.sendAction('PLACE');
    expect(commands, isEmpty);
    await model.sendAction('GRAB');
    expect(commands, ['GRAB']);
    expect(model.canPickupAction('GRAB'), isFalse);
    expect(model.canPickupAction('RESET'), isFalse);
    expect(model.status!.progress, .2);
    status = pickupStatus('HOLDING', ['PLACE', 'STOP', 'RESET']);
    await model.refresh();
    expect(model.status!.holding, isTrue);
    expect(model.canPickupAction('PLACE'), isTrue);
    status = pickupStatus('STOPPED', ['RESUME', 'STOP', 'RESET']);
    await model.refresh();
    expect(model.canPickupAction('RESUME'), isTrue);
    model.setPaused(true);
    expect(model.canPickupAction('RESUME'), isFalse);
  });

  test('choosing a can updates the authoritative selection and is blocked during pickup', () async {
    var status = pickupStatus('READY', ['GRAB', 'RESET', 'STOP']);
    final selected = <String>[];
    final model = DashboardController(api: RobotApi(client: MockClient((request) async {
      if (request.url.path == '/robot/can') {
        final value = (jsonDecode(request.body) as Map)['canType'] as String;
        selected.add(value);
        status = {...status, 'selectedCan': value};
        return jsonResponse({'success': true, 'selectedCan': value, 'message': 'Selected'});
      }
      return jsonResponse(request.url.path.endsWith('status') ? status : telemetryData);
    })));
    addTearDown(model.dispose);
    await model.refresh();
    expect(model.canSelectCan, isTrue);
    expect(model.status!.canTypes.length, 4);
    await model.selectCan('tuna');
    expect(selected, ['tuna']);
    expect(model.status!.selectedCan, 'tuna');
    await model.selectCan('unknown');
    expect(selected, ['tuna']);
    status = pickupStatus('GRABBING', ['STOP']);
    await model.refresh();
    expect(model.canSelectCan, isFalse);
    await model.selectCan('pepsi');
    expect(selected, ['tuna']);
    status = pickupStatus('HOLDING', ['PLACE', 'STOP']);
    await model.refresh();
    expect(model.canSelectCan, isFalse);
  });

  test('scan uses action endpoint and detection stays separate from scene selection', () async {
    var data = pickupStatus('READY', ['SCAN', 'GRAB', 'STOP']);
    String? sent;
    final model = DashboardController(api: RobotApi(client: MockClient((request) async {
      if (request.method == 'POST') {
        sent = (jsonDecode(request.body) as Map)['action'] as String;
        data = {...data, 'vision': {'status':'DETECTED', 'detectedCan':'pepsi',
          'confidence':.98, 'position':[.4,-.18,.91], 'frameId':4,
          'detections':[{'detectedCan':'cola'}, {'detectedCan':'pepsi'}]}};
        return jsonResponse({'success':true});
      }
      return jsonResponse(request.url.path.endsWith('status') ? data : telemetryData);
    })));
    addTearDown(model.dispose);
    await model.refresh();
    expect(model.canPickupAction('SCAN'), isTrue);
    await model.sendAction('SCAN');
    expect(sent, 'SCAN');
    expect(model.status!.selectedCan, 'cola');
    expect(model.status!.vision['detectedCan'], 'pepsi');
    expect(model.status!.vision['frameId'], 4);
    expect((model.status!.vision['detections'] as List).length, 2);
  });

  test('invalid pickup permissions and progress are rejected', () {
    final data = pickupStatus('READY', ['GRAB']);
    for (final invalid in [
      {...data, 'availableActions': [1]},
      {...data, 'availableActions': 'GRAB'},
      {...data, 'progress': 1.5},
      {...data, 'holding': 'yes'},
      {...data, 'vision': {'confidence':1.5}},
      {...data, 'vision': {'position':['bad',0,0]}},
      {...data, 'vision': {'frameId':'bad'}},
      {...data, 'vision': {'detections':[1]}},
      {...data, 'selectedCan': 'unknown'},
      {...data, 'canTypes': [{'id':1, 'label':'Bad'}]},
      {...data, 'canSelectionAllowed': 'yes'},
      {...data, 'canTypes': [{'id':'cola','label':'A'}, {'id':'cola','label':'B'}]},
    ]) {
      expect(() => RobotStatus.fromJson(invalid), throwsFormatException);
    }
  });

  for (final size in [const Size(320, 640), const Size(390, 844)]) {
    testWidgets('pickup dashboard fits $size and enables only valid commands', (tester) async {
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      final model = DashboardController(api: RobotApi(client: MockClient((request) async {
        return jsonResponse(request.url.path.endsWith('status')
            ? pickupStatus('READY', ['GRAB', 'RESET', 'STOP']) : telemetryData);
      })));
      await tester.pumpWidget(MotionCoreApp(home: HomePage(controller: model)));
      await tester.pump();
      final selector = tester.widget<DropdownButton<String>>(find.byKey(const Key('can-selector')));
      expect(selector.items!.length, 4);
      expect(selector.value, 'cola');
      expect(selector.onChanged, isNotNull);
      expect(find.byKey(const Key('walk-button')), findsNothing);
      expect(find.byKey(const Key('speed-slider')), findsNothing);
      expect(tester.widget<FilledButton>(find.byKey(const Key('grab-button'))).onPressed, isNotNull);
      expect(tester.widget<FilledButton>(find.byKey(const Key('place-button'))).onPressed, isNull);
      expect(find.byKey(const Key('stop-button')), findsOneWidget);
      await tester.ensureVisible(find.byKey(const Key('reset-scene-button')));
      await tester.pump();
      expect(tester.takeException(), isNull);
      await tester.pumpWidget(const SizedBox());
      model.dispose();
    });
  }
}
