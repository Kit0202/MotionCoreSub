import 'dart:async';
import 'dart:convert';

import 'package:http/http.dart' as http;

class RobotApiException implements Exception {
  const RobotApiException(this.message);
  final String message;
  @override
  String toString() => message;
}

class RobotApi {
  RobotApi({
    String baseUrl = defaultBaseUrl,
    http.Client? client,
    this.timeout = const Duration(seconds: 4),
  }) : _client = client ?? http.Client(),
       _baseUrl = normalizeBaseUrl(baseUrl);

  static const defaultBaseUrl = String.fromEnvironment(
    'ROBOT_API_URL',
    defaultValue: 'http://10.200.233.20:8000',
  );
  final http.Client _client;
  final Duration timeout;
  String _baseUrl;
  String get baseUrl => _baseUrl;
  set baseUrl(String value) => _baseUrl = normalizeBaseUrl(value);

  static String normalizeBaseUrl(String value) {
    final uri = Uri.tryParse(value.trim());
    if (uri == null ||
        !['http', 'https'].contains(uri.scheme) ||
        uri.host.isEmpty ||
        uri.userInfo.isNotEmpty ||
        uri.hasQuery ||
        uri.hasFragment ||
        (uri.path.isNotEmpty && uri.path != '/')) {
      throw const FormatException(
        'Enter a server address such as http://192.168.1.10:8000',
      );
    }
    // Accessing port also validates malformed port values.
    if (uri.port < 1 || uri.port > 65535) {
      throw const FormatException('Enter a port between 1 and 65535');
    }
    return uri.replace(path: '').toString();
  }

  Future<Map<String, dynamic>> _request(
    String path, {
    bool post = false,
    Map<String, dynamic>? body,
  }) async {
    try {
      final uri = Uri.parse('$baseUrl$path');
      final response =
          await (post
                  ? _client.post(
                      uri,
                      headers: {'Content-Type': 'application/json'},
                      body: body == null ? null : jsonEncode(body),
                    )
                  : _client.get(uri))
              .timeout(timeout);
      if (response.statusCode < 200 || response.statusCode >= 300) {
        throw RobotApiException('Server returned HTTP ${response.statusCode}');
      }
      final data = jsonDecode(response.body);
      if (data is! Map<String, dynamic>) {
        throw const FormatException('Expected a JSON object');
      }
      if (post && data['success'] != true) {
        throw RobotApiException(
          data['message'] is String
              ? data['message'] as String
              : 'The server did not confirm the command',
        );
      }
      return data;
    } on TimeoutException {
      throw const RobotApiException(
        'Request timed out. Check the server connection.',
      );
    } on http.ClientException {
      throw const RobotApiException(
        'Cannot reach the server. Check Wi-Fi and the server address.',
      );
    } on FormatException {
      throw const RobotApiException('The server returned an invalid response.');
    }
  }

  Future<Map<String, dynamic>> getRobotStatus() => _request('/robot/status');
  Future<Map<String, dynamic>> getTelemetry() => _request('/robot/telemetry');
  Future<Map<String, dynamic>> sendCommand(String action, double speed) =>
      _request(
        '/robot/action',
        post: true,
        body: {'action': action, 'speed': speed},
      );
  Future<Map<String, dynamic>> changeSpeed(double speed) =>
      _request('/robot/speed', post: true, body: {'speed': speed});
  Future<Map<String, dynamic>> stopRobot() =>
      _request('/robot/stop', post: true);
  void close() => _client.close();
}
