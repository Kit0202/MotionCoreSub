class RobotTelemetry {
  const RobotTelemetry({
    required this.height,
    required this.roll,
    required this.pitch,
    required this.leftFoot,
    required this.rightFoot,
    required this.fallen,
    this.measuredSpeed,
    this.commandSpeed,
  });

  final double height;
  final double roll;
  final double pitch;
  final bool leftFoot;
  final bool rightFoot;
  final bool fallen;
  final double? measuredSpeed;
  final double? commandSpeed;

  factory RobotTelemetry.fromJson(Map<String, dynamic> json) {
    double number(String key) {
      final value = json[key];
      if (value is! num || !value.isFinite) {
        throw FormatException('Invalid telemetry: $key');
      }
      return value.toDouble();
    }

    bool contact(String key) {
      final value = json[key];
      if (value is! bool) throw FormatException('Invalid telemetry: $key');
      return value;
    }

    final height = number('height');
    // The existing backend returns zero height when its file cannot be read.
    // Never turn that fallback into a reassuring stability indication.
    if (height <= 0 || json['available'] == false) {
      throw const FormatException('No simulator sample available');
    }
    return RobotTelemetry(
      height: height,
      roll: number('roll'),
      pitch: number('pitch'),
      leftFoot: contact('leftFoot'),
      rightFoot: contact('rightFoot'),
      fallen: contact('fallen'),
      measuredSpeed: json['measuredSpeed'] == null ? null : number('measuredSpeed'),
      commandSpeed: json['commandSpeed'] == null ? null : number('commandSpeed'),
    );
  }
}
