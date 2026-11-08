import 'package:flutter/material.dart';

import 'screens/home_page.dart';

void main() => runApp(const MotionCoreApp());

class MotionCoreApp extends StatelessWidget {
  const MotionCoreApp({super.key, this.home});
  final Widget? home;

  @override
  Widget build(BuildContext context) {
    final scheme = ColorScheme.fromSeed(
      seedColor: const Color(0xFF63E2C0),
      brightness: Brightness.dark,
      surface: const Color(0xFF131E2C),
    );
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'MotionCore',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: scheme.copyWith(primary: const Color(0xFF63E2C0)),
        scaffoldBackgroundColor: const Color(0xFF0B121C),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFF0B121C),
          foregroundColor: Color(0xFFF2F6FA),
          surfaceTintColor: Colors.transparent,
        ),
        textTheme: ThemeData.dark().textTheme.apply(
          bodyColor: const Color(0xFFEAF1F8),
          displayColor: const Color(0xFFEAF1F8),
        ),
        sliderTheme: const SliderThemeData(
          trackHeight: 5,
          showValueIndicator: ShowValueIndicator.onDrag,
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: FilledButton.styleFrom(
            minimumSize: const Size(48, 52),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(16),
            ),
            textStyle: ThemeData.dark().textTheme.labelLarge!.copyWith(
              fontWeight: FontWeight.w700,
              fontSize: 15,
            ),
          ),
        ),
      ),
      home: home ?? const HomePage(),
    );
  }
}
