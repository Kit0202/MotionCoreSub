import 'dart:async';

import 'package:flutter/foundation.dart';

import '../models/robot_status.dart';
import '../models/robot_telemetry.dart';
import '../services/robot_api.dart';

class DashboardController extends ChangeNotifier {
  DashboardController({RobotApi? api, DateTime Function()? now})
    : api = api ?? RobotApi(),
      _now = now ?? DateTime.now;

  final RobotApi api;
  final DateTime Function() _now;
  RobotStatus? status;
  RobotTelemetry? telemetry;
  DateTime? statusReceivedAt;
  DateTime? telemetryReceivedAt;
  String? statusError;
  String? telemetryError;
  String? commandMessage;
  bool commandFailed = false;
  double requestedSpeed = 0.5;
  String? pendingAction;
  bool stopping = false;
  bool _disposed = false;
  bool _paused = false;
  bool _started = false;
  int _generation = 0;
  int _statusRevision = 0;
  Timer? _timer;
  Future<void>? _refresh;
  Future<void>? _action;

  bool get statusFresh => statusError == null && _fresh(statusReceivedAt);
  bool get telemetryFresh =>
      telemetryError == null && _fresh(telemetryReceivedAt);
  bool get connected => statusFresh && status?.connected == true;
  bool get busy => pendingAction != null || stopping;
  bool get remoteBusy =>
      statusFresh &&
      const {'STARTING', 'MOVING', 'WALKING', 'STANDING', 'STOPPING'}.contains(status?.status);
  bool get canWalk =>
      connected &&
      !busy &&
      !remoteBusy &&
      !(telemetryFresh && telemetry?.fallen == true);
  bool get canDemoAction =>
      connected &&
      !busy &&
      !remoteBusy;
  bool _fresh(DateTime? time) =>
      time != null &&
      _now().difference(time) < const Duration(seconds: 6) &&
      !_paused;

  String age(DateTime? time) {
    if (time == null) return 'Not received';
    final seconds = _now().difference(time).inSeconds;
    return seconds < 1 ? 'Just now' : '${seconds}s ago';
  }

  void _emit() {
    if (!_disposed) notifyListeners();
  }

  void start() {
    if (_started || _disposed) return;
    _started = true;
    _paused = false;
    unawaited(refresh());
    _timer = Timer.periodic(const Duration(seconds: 1), (_) {
      if (!_paused) {
        _emit();
        unawaited(refresh());
      }
    });
  }

  void setPaused(bool value) {
    _paused = value;
    _emit();
    if (!value) unawaited(refresh());
  }

  Future<void> refresh() {
    if (_disposed || _paused) return Future.value();
    return _refresh ??= _poll().whenComplete(() => _refresh = null);
  }

  Future<void> _poll() async {
    final generation = _generation;
    final revision = _statusRevision;
    await Future.wait([
      _readStatus(generation, revision),
      _readTelemetry(generation),
    ]);
  }

  Future<void> _readStatus(int generation, int revision) async {
    try {
      final value = RobotStatus.fromJson(await api.getRobotStatus());
      if (_disposed ||
          generation != _generation ||
          revision != _statusRevision) {
        return;
      }
      status = value;
      statusReceivedAt = _now();
      statusError = null;
    } catch (error) {
      if (_disposed ||
          generation != _generation ||
          revision != _statusRevision) {
        return;
      }
      statusError = error.toString();
    }
    _emit();
  }

  Future<void> _readTelemetry(int generation) async {
    try {
      final value = RobotTelemetry.fromJson(await api.getTelemetry());
      if (_disposed || generation != _generation) return;
      telemetry = value;
      telemetryReceivedAt = _now();
      telemetryError = null;
    } catch (error) {
      if (_disposed || generation != _generation) return;
      telemetryError = error.toString();
    }
    _emit();
  }

  void setSpeed(double value) {
    requestedSpeed = value.clamp(0, 1);
    _emit();
  }

  Future<void> applySpeed() async {
    // The initial value is sent with WALK; active walking accepts live updates.
    if (busy || !connected || status?.status != 'WALKING') return;
    pendingAction = 'SPEED';
    _statusRevision++;
    _emit();
    _action = _performSpeed();
    await _action;
    _action = null;
    if (!_disposed) {
      await _refresh;
      await refresh();
    }
    pendingAction = null;
    _emit();
  }

  Future<void> _performSpeed() async {
    try {
      final data = await api.changeSpeed(requestedSpeed);
      if (!stopping && !_disposed) {
        commandFailed = false;
        commandMessage = data['message'] as String? ?? 'Speed sent';
      }
    } catch (error) {
      if (!stopping && !_disposed) {
        commandFailed = true;
        commandMessage = '$error Speed update was not confirmed.';
      }
    }
  }

  Future<void> setServer(String url) async {
    if (busy) return;
    api.baseUrl = url;
    _generation++;
    status = null;
    telemetry = null;
    statusReceivedAt = telemetryReceivedAt = null;
    statusError = telemetryError = commandMessage = null;
    _emit();
    // Finish old requests before starting a poll for the new server.
    await _refresh;
    if (!_disposed) await refresh();
  }

  Future<void> sendAction(String action) async {
    if (busy || !connected) return;
    if (action == 'WALK' && !canWalk) return;
    if (action != 'WALK' && !canDemoAction) return;
    pendingAction = action;
    commandMessage = null;
    _statusRevision++;
    _emit();
    _action = _performAction(action);
    await _action;
    _action = null;
    if (!_disposed) {
      await _refresh;
      await refresh();
    }
    pendingAction = null;
    _emit();
  }

  Future<void> _performAction(String action) async {
    try {
      final data = await api.sendCommand(action, requestedSpeed);
      if (!stopping && !_disposed) {
        commandFailed = false;
        commandMessage = data['message'] as String? ?? '$action accepted';
      }
    } catch (error) {
      if (!stopping && !_disposed) {
        commandFailed = true;
        commandMessage =
            '$error Command outcome is unconfirmed; check status before retrying.';
      }
    }
  }

  Future<void> stop() async {
    if (stopping || _disposed) return;
    stopping = true;
    commandMessage = null;
    _statusRevision++;
    _emit();
    // Preserve command order: a pending WALK must not be sent after STOP.
    await _action;
    if (_disposed) return;
    try {
      final data = await api.stopRobot();
      commandFailed = false;
      commandMessage = data['message'] as String? ?? 'Stop accepted';
    } catch (error) {
      commandFailed = true;
      commandMessage = 'Stop was not confirmed. $error';
    } finally {
      if (!_disposed) {
        await _refresh;
        await refresh();
      }
      stopping = false;
      _emit();
    }
  }

  @override
  void dispose() {
    _disposed = true;
    _timer?.cancel();
    api.close();
    super.dispose();
  }
}
