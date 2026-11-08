class RobotStatus {
  const RobotStatus({
    required this.status,
    required this.motion,
    required this.speed,
    required this.connected,
    this.controller = 'NONE',
    this.message,
  });

  final String status;
  final String motion;
  final double speed;
  final bool connected;
  final String controller;
  final String? message;

  factory RobotStatus.fromJson(Map<String, dynamic> json) {
    final speed = json['speed'];
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
    );
  }
}
