import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:motioncore_mobile/controllers/dashboard_controller.dart';
import 'package:motioncore_mobile/main.dart';
import 'package:motioncore_mobile/screens/home_page.dart';
import 'package:motioncore_mobile/services/robot_api.dart';

import 'dashboard_controller_test.dart'
    show statusData, telemetryData, jsonResponse;

void main() {
  for (final size in [
    const Size(390, 844),
    const Size(320, 640),
    const Size(1100, 850),
  ]) {
    testWidgets('dashboard fits $size and displays real sample values', (
      tester,
    ) async {
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      final model = DashboardController(
        api: RobotApi(
          client: MockClient(
            (request) async => jsonResponse(
              request.url.path.endsWith('status') ? statusData : telemetryData,
            ),
          ),
        ),
      );
      await tester.pumpWidget(MotionCoreApp(home: HomePage(controller: model)));
      await tester.pump();
      expect(find.textContaining('API connected'), findsOneWidget);
      expect(find.text('0.773'), findsOneWidget);
      expect(find.text('-0.009'), findsOneWidget);
      expect(find.text('-0.059'), findsOneWidget);
      expect(find.text('CONTACT'), findsOneWidget);
      expect(find.text('AIR'), findsOneWidget);
      expect(tester.takeException(), isNull);
      expect(
        tester.getBottomRight(find.byKey(const Key('stop-button'))).dy,
        lessThanOrEqualTo(size.height),
      );
      await tester.pumpWidget(const SizedBox());
      model.dispose();
    });
  }

  testWidgets(
    'large text fits narrow phone and disconnected values are unknown',
    (tester) async {
      tester.view.physicalSize = const Size(320, 640);
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      final model = DashboardController(
        api: RobotApi(client: MockClient((_) async => http.Response('', 503))),
      );
      await tester.pumpWidget(
        MotionCoreApp(
          home: MediaQuery(
            data: const MediaQueryData(textScaler: TextScaler.linear(2)),
            child: HomePage(controller: model),
          ),
        ),
      );
      await tester.pump();
      expect(find.textContaining('API disconnected'), findsOneWidget);
      expect(find.text('UNKNOWN'), findsNWidgets(2));
      expect(
        tester
            .widget<FilledButton>(find.byKey(const Key('walk-button')))
            .onPressed,
        isNull,
      );
      expect(
        tester
            .widget<FilledButton>(find.byKey(const Key('stop-button')))
            .onPressed,
        isNotNull,
      );
      expect(tester.takeException(), isNull);
      await tester.pumpWidget(const SizedBox());
      model.dispose();
    },
  );

  testWidgets('fall indication disables WALK without hiding STOP', (
    tester,
  ) async {
    final model = DashboardController(
      api: RobotApi(
        client: MockClient(
          (request) async => jsonResponse(
            request.url.path.endsWith('status')
                ? statusData
                : {...telemetryData, 'fallen': true},
          ),
        ),
      ),
    );
    await tester.pumpWidget(MotionCoreApp(home: HomePage(controller: model)));
    await tester.pump();
    expect(find.textContaining('Fall detected'), findsOneWidget);
    expect(
      tester
          .widget<FilledButton>(find.byKey(const Key('walk-button')))
          .onPressed,
      isNull,
    );
    expect(
      tester
          .widget<FilledButton>(find.byKey(const Key('stop-button')))
          .onPressed,
      isNotNull,
    );
    await tester.pumpWidget(const SizedBox());
    model.dispose();
  });
}
