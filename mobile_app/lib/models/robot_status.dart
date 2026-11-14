class CanOption {
  const CanOption({required this.id, required this.label});
  final String id;
  final String label;

  factory CanOption.fromJson(Map<String, dynamic> json) {
    if (json['id'] is! String || json['label'] is! String ||
        (json['id'] as String).isEmpty || (json['label'] as String).isEmpty) {
      throw const FormatException('Invalid can option');
    }
    return CanOption(id: json['id'] as String, label: json['label'] as String);
  }
}

class RobotStatus {
  const RobotStatus({
    required this.status,
    required this.motion,
    required this.speed,
    required this.connected,
    this.controller = 'NONE',
    this.message,
    this.scene = 'motion',
    this.availableActions,
    this.holding = false,
    this.progress,
    this.selectedCan,
    this.canTypes = const [],
    this.canSelectionAllowed = false,
    this.vision = const {},
  });

  final String status;
  final String motion;
  final double speed;
  final bool connected;
  final String controller;
  final String? message;
  final String scene;
  final List<String>? availableActions;
  final bool holding;
  final double? progress;
  final String? selectedCan;
  final List<CanOption> canTypes;
  final bool canSelectionAllowed;
  final Map<String, dynamic> vision;

  factory RobotStatus.fromJson(Map<String, dynamic> json) {
    final speed = json['speed'];
    final actions = json['availableActions'];
    final progress = json['progress'];
    final options = <CanOption>[];
    final rawOptions = json['canTypes'];
    if (rawOptions != null) {
      if (rawOptions is! List) throw const FormatException('Invalid can options');
      for (final option in rawOptions) {
        if (option is! Map<String, dynamic>) throw const FormatException('Invalid can option');
        options.add(CanOption.fromJson(option));
      }
      if (options.map((o) => o.id).toSet().length != options.length) {
        throw const FormatException('Duplicate can options');
      }
    }
    final rawVision = json['vision'];
    if (rawVision != null) {
      if (rawVision is! Map<String, dynamic>) throw const FormatException('Invalid camera result');
      final confidence = rawVision['confidence'];
      final position = rawVision['position'];
      final frame = rawVision['frameId'];
      final detections = rawVision['detections'];
      if (detections != null && (detections is! List || detections.any((d) =>
          d is! Map<String, dynamic> ||
          (d['detectedCan'] != null && d['detectedCan'] is! String)))) {
        throw const FormatException('Invalid camera detections');
      }

      if ((rawVision['status'] != null && rawVision['status'] is! String) ||
          (rawVision['message'] != null && rawVision['message'] is! String) ||
          (rawVision['detectedCan'] != null && rawVision['detectedCan'] is! String) ||
          (confidence != null && (confidence is! num || !confidence.isFinite || confidence < 0 || confidence > 1)) ||
          (position != null && (position is! List || position.length != 3 ||
            position.any((v) => v is! num || !v.isFinite))) ||
          (frame != null && (frame is! int || frame < 0))) {
        throw const FormatException('Invalid camera result');
      }
    }
    final selectedCan = json['selectedCan'];
    if ((selectedCan != null && (selectedCan is! String || !options.any((o) => o.id == selectedCan))) ||
        (json['canSelectionAllowed'] != null && json['canSelectionAllowed'] is! bool)) {
      throw const FormatException('Invalid can selection');
    }
    if ((actions != null && (actions is! List || actions.any((a) => a is! String))) ||
        (json['scene'] != null && json['scene'] is! String) ||
        (json['holding'] != null && json['holding'] is! bool) ||
        (progress != null && (progress is! num || !progress.isFinite || progress < 0 || progress > 1))) {
      throw const FormatException('Invalid scene status');
    }
    if (json['status'] is! String ||
        json['motion'] is! String ||
        json['connected'] is! bool ||
        speed is! num ||
        !speed.isFinite ||
        (json['controller'] != null && json['controller'] is! String)) {
      throw const FormatException('Invalid robot status');
    }
    return RobotStatus(
      status: json['status'] as String,
      motion: json['motion'] as String,
      speed: speed.toDouble(),
      connected: json['connected'] as bool,
      controller: json['controller'] as String? ?? 'NONE',
      message: json['message'] is String ? json['message'] as String : null,
      scene: json['scene'] as String? ?? 'motion',
      vision: json['vision'] is Map<String, dynamic>
          ? Map<String, dynamic>.unmodifiable(json['vision'] as Map<String, dynamic>) : const {},
      availableActions: actions == null ? null : List<String>.unmodifiable(actions as List),
      holding: json['holding'] as bool? ?? false,
      progress: (progress as num?)?.toDouble(),
      selectedCan: selectedCan as String?,
      canTypes: List<CanOption>.unmodifiable(options),
      canSelectionAllowed: json['canSelectionAllowed'] as bool? ?? false,
    );
  }
}
