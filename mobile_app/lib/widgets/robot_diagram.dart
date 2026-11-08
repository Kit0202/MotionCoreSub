import 'dart:math' as math;

import 'package:flutter/material.dart';

/// A schematic, not a camera feed or a reconstruction of joint telemetry.
class RobotDiagram extends StatelessWidget {
  const RobotDiagram({
    super.key,
    this.leftFoot,
    this.rightFoot,
    this.fallen = false,
  });
  final bool? leftFoot;
  final bool? rightFoot;
  final bool fallen;

  @override
  Widget build(BuildContext context) => Semantics(
    label: 'G1 schematic. Foot highlights show the latest contact sample.',
    child: CustomPaint(
      size: const Size(220, 245),
      painter: _RobotPainter(leftFoot, rightFoot, fallen),
    ),
  );
}

class _RobotPainter extends CustomPainter {
  const _RobotPainter(this.left, this.right, this.fallen);
  final bool? left;
  final bool? right;
  final bool fallen;

  @override
  void paint(Canvas canvas, Size size) {
    canvas.save();
    canvas.scale(size.width / 220, size.height / 245);
    final accent = fallen ? const Color(0xFFFF8990) : const Color(0xFF63E2C0);
    final grid = Paint()
      ..color = const Color(0xFF223040)
      ..strokeWidth = 0.7;
    for (double x = 10; x < 220; x += 20) {
      canvas.drawLine(Offset(x, 0), Offset(x, 230), grid);
    }
    for (double y = 10; y < 230; y += 20) {
      canvas.drawLine(Offset(10, y), Offset(210, y), grid);
    }
    canvas.drawCircle(
      const Offset(110, 112),
      90,
      Paint()..color = accent.withValues(alpha: 0.07),
    );
    canvas.drawArc(
      const Rect.fromLTWH(20, 22, 180, 180),
      -math.pi / 2,
      math.pi * 1.4,
      false,
      Paint()
        ..color = accent.withValues(alpha: 0.25)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );
    void segment(Offset a, Offset b, double width) {
      canvas.drawLine(
        a,
        b,
        Paint()
          ..color = const Color(0xFF91A5B8)
          ..strokeWidth = width
          ..strokeCap = StrokeCap.round,
      );
      canvas.drawLine(
        a,
        b,
        Paint()
          ..color = const Color(0xFFCBD8E3)
          ..strokeWidth = width - 5
          ..strokeCap = StrokeCap.round,
      );
    }

    segment(const Offset(91, 138), const Offset(83, 177), 18);
    segment(const Offset(83, 177), const Offset(83, 212), 15);
    segment(const Offset(129, 138), const Offset(137, 177), 18);
    segment(const Offset(137, 177), const Offset(137, 212), 15);
    segment(const Offset(74, 76), const Offset(58, 112), 14);
    segment(const Offset(58, 112), const Offset(63, 143), 12);
    segment(const Offset(146, 76), const Offset(162, 112), 14);
    segment(const Offset(162, 112), const Offset(157, 143), 12);
    final body = Path()
      ..moveTo(78, 65)
      ..lineTo(142, 65)
      ..lineTo(135, 113)
      ..lineTo(128, 128)
      ..lineTo(92, 128)
      ..lineTo(85, 113)
      ..close();
    canvas.drawPath(
      body,
      Paint()
        ..shader = const LinearGradient(
          colors: [Color(0xFFE0EAF1), Color(0xFF8094A8)],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ).createShader(const Rect.fromLTWH(78, 65, 64, 65)),
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        const Rect.fromLTWH(93, 78, 34, 25),
        const Radius.circular(7),
      ),
      Paint()..color = const Color(0xFF172536),
    );
    canvas.drawLine(
      const Offset(101, 91),
      const Offset(119, 91),
      Paint()
        ..color = accent
        ..strokeWidth = 3
        ..strokeCap = StrokeCap.round,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        const Rect.fromLTWH(92, 29, 36, 29),
        const Radius.circular(11),
      ),
      Paint()..color = const Color(0xFFCDDCE7),
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        const Rect.fromLTWH(96, 39, 28, 9),
        const Radius.circular(4),
      ),
      Paint()..color = const Color(0xFF172536),
    );
    canvas.drawCircle(const Offset(118, 43), 2, Paint()..color = accent);
    for (final point in [
      const Offset(83, 177),
      const Offset(137, 177),
      const Offset(58, 112),
      const Offset(162, 112),
    ]) {
      canvas.drawCircle(point, 5, Paint()..color = const Color(0xFF273B4F));
      canvas.drawCircle(point, 2, Paint()..color = accent);
    }
    void foot(double x, bool? contact) {
      final color = contact == true ? accent : const Color(0xFF65798E);
      canvas.drawOval(
        Rect.fromCenter(center: Offset(x, 227), width: 48, height: 12),
        Paint()..color = color.withValues(alpha: 0.16),
      );
      canvas.drawRRect(
        RRect.fromRectAndRadius(
          Rect.fromLTWH(x - 14, 210, 28, 12),
          const Radius.circular(4),
        ),
        Paint()..color = color,
      );
    }

    foot(83, left);
    foot(137, right);
    canvas.restore();
  }

  @override
  bool shouldRepaint(_RobotPainter oldDelegate) =>
      left != oldDelegate.left ||
      right != oldDelegate.right ||
      fallen != oldDelegate.fallen;
}
