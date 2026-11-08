import 'dart:async';

import 'package:flutter/material.dart';

import '../controllers/dashboard_controller.dart';
import '../services/robot_api.dart';
import '../widgets/robot_diagram.dart';

const _mint = Color(0xFF63E2C0);
const _muted = Color(0xFF99AABD);
const _amber = Color(0xFFF5CA7A);
const _red = Color(0xFFFF8990);

class HomePage extends StatefulWidget {
  const HomePage({super.key, this.controller});
  final DashboardController? controller;
  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with WidgetsBindingObserver {
  late final DashboardController model;

  @override
  void initState() {
    super.initState();
    model = widget.controller ?? DashboardController();
    WidgetsBinding.instance.addObserver(this);
    model.start();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    model.setPaused(state != AppLifecycleState.resumed);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    if (widget.controller == null) model.dispose();
    super.dispose();
  }

  Future<void> _serverSettings() async {
    final input = TextEditingController(text: model.api.baseUrl);
    final formKey = GlobalKey<FormState>();
    final result = await showDialog<String>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Server connection'),
        content: SingleChildScrollView(
          child: Form(
            key: formKey,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text(
                  'Use the laptop address on the same Wi-Fi network as your phone.',
                ),
                const SizedBox(height: 20),
                TextFormField(
                  controller: input,
                  keyboardType: TextInputType.url,
                  autocorrect: false,
                  decoration: const InputDecoration(
                    labelText: 'Server URL',
                    hintText: 'http://192.168.1.10:8000',
                    border: OutlineInputBorder(),
                  ),
                  validator: (value) {
                    try {
                      RobotApi.normalizeBaseUrl(value ?? '');
                      return null;
                    } on FormatException catch (error) {
                      return error.message;
                    }
                  },
                ),
                const SizedBox(height: 12),
                const Text(
                  'Changes apply for this app session.',
                  style: TextStyle(color: _muted, fontSize: 12),
                ),
              ],
            ),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          FilledButton(
            onPressed: () {
              if (formKey.currentState!.validate()) {
                Navigator.pop(context, input.text);
              }
            },
            child: const Text('Connect'),
          ),
        ],
      ),
    );
    // The dialog's closing transition still owns its text field briefly.
    await Future<void>.delayed(const Duration(milliseconds: 300));
    input.dispose();
    if (result != null && mounted) await model.setServer(result);
  }

  @override
  Widget build(BuildContext context) => AnimatedBuilder(
    animation: model,
    builder: (context, _) => Scaffold(
      appBar: AppBar(
        toolbarHeight: 76,
        titleSpacing: 20,
        title: Row(
          children: [
            Container(
              width: 36,
              height: 36,
              decoration: BoxDecoration(
                color: _mint,
                borderRadius: BorderRadius.circular(11),
              ),
              child: const Icon(
                Icons.hub_outlined,
                color: Color(0xFF0B121C),
                size: 23,
              ),
            ),
            const SizedBox(width: 10),
            const Flexible(
              child: Text(
                'MotionCore',
                style: TextStyle(
                  fontWeight: FontWeight.w800,
                  fontSize: 21,
                  letterSpacing: -0.6,
                ),
              ),
            ),
          ],
        ),
        actions: [
          IconButton(
            tooltip: 'Server connection',
            onPressed: model.busy ? null : _serverSettings,
            icon: const Icon(Icons.tune_rounded),
          ),
          const SizedBox(width: 10),
        ],
      ),
      bottomNavigationBar: _stopBar(),
      body: RefreshIndicator(
        onRefresh: model.refresh,
        child: SingleChildScrollView(
          physics: const AlwaysScrollableScrollPhysics(),
          padding: const EdgeInsets.fromLTRB(20, 2, 20, 24),
          child: Align(
            alignment: Alignment.topCenter,
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 1000),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  _connection(),
                  const SizedBox(height: 25),
                  const Text(
                    'Humanoid control',
                    style: TextStyle(
                      fontSize: 29,
                      fontWeight: FontWeight.w700,
                      letterSpacing: -0.8,
                    ),
                  ),
                  const SizedBox(height: 5),
                  const Text(
                    'Your G1, at a glance.',
                    style: TextStyle(color: _muted, fontSize: 15),
                  ),
                  const SizedBox(height: 20),
                  _robotCard(),
                  const SizedBox(height: 26),
                  LayoutBuilder(
                    builder: (context, constraints) {
                      if (constraints.maxWidth >= 800) {
                        return Row(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Expanded(child: _telemetrySection()),
                            const SizedBox(width: 22),
                            Expanded(child: _controlsSection()),
                          ],
                        );
                      }
                      return Column(
                        children: [
                          _telemetrySection(),
                          const SizedBox(height: 26),
                          _controlsSection(),
                        ],
                      );
                    },
                  ),
                  const SizedBox(height: 24),
                  const Center(
                    child: Text(
                      'MOTIONCORE  /  G1 RESEARCH PLATFORM',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        color: _muted,
                        fontSize: 10,
                        letterSpacing: 1.7,
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    ),
  );

  Widget _connection() {
    final waiting = model.status == null && model.statusError == null;
    final label = model.connected
        ? 'API connected'
        : waiting
        ? 'Connecting'
        : 'API disconnected';
    final color = model.connected ? _mint : _amber;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Wrap(
          spacing: 10,
          runSpacing: 8,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            _badge(label, color),
            const Text(
              'MuJoCo simulation',
              style: TextStyle(color: _muted, fontSize: 12),
            ),
          ],
        ),
        if (model.statusError != null)
          Padding(
            padding: const EdgeInsets.only(top: 12),
            child: _notice(
              '${model.statusError}\nRetrying automatically. Check the server address in settings.',
              _amber,
            ),
          ),
      ],
    );
  }

  Widget _robotCard() {
    final status = model.status;
    final valueColor = model.statusFresh ? _mint : _muted;
    return _panel(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Wrap(
            spacing: 10,
            runSpacing: 8,
            crossAxisAlignment: WrapCrossAlignment.center,
            children: [
              const Text(
                'UNITREE',
                style: TextStyle(
                  color: _muted,
                  fontSize: 11,
                  letterSpacing: 2.5,
                  fontWeight: FontWeight.w700,
                ),
              ),
              _badge(
                model.statusFresh
                    ? status!.status.replaceAll('_', ' ')
                    : status == null
                    ? 'AWAITING STATUS'
                    : 'LAST KNOWN STATUS',
                valueColor,
              ),
            ],
          ),
          const SizedBox(height: 14),
          LayoutBuilder(
            builder: (context, constraints) {
              final compact =
                  constraints.maxWidth < 280 ||
                  MediaQuery.textScalerOf(context).scale(1) > 1.4;
              final details = Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    'G1',
                    style: TextStyle(
                      fontSize: 58,
                      height: 1,
                      letterSpacing: -3,
                      fontWeight: FontWeight.w800,
                    ),
                  ),
                  const SizedBox(height: 9),
                  const Text(
                    'Humanoid robot',
                    style: TextStyle(color: _muted, fontSize: 12),
                  ),
                  const SizedBox(height: 22),
                  _detail('MOTION', status?.motion.replaceAll('_', ' ') ?? '—'),
                  const SizedBox(height: 13),
                  _detail(
                    'CONTROLLER',
                    status?.controller.replaceAll('_', ' ') ?? '—',
                  ),
                ],
              );
              final figure = SizedBox(
                width: 180,
                height: 225,
                child: RobotDiagram(
                  leftFoot: model.telemetryFresh
                      ? model.telemetry?.leftFoot
                      : null,
                  rightFoot: model.telemetryFresh
                      ? model.telemetry?.rightFoot
                      : null,
                  fallen:
                      model.telemetryFresh && model.telemetry?.fallen == true,
                ),
              );
              return compact
                  ? Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        details,
                        Center(child: figure),
                      ],
                    )
                  : Row(
                      children: [
                        Expanded(child: details),
                        figure,
                      ],
                    );
            },
          ),
          const SizedBox(height: 8),
          const Divider(color: Color(0xFF2A3746)),
          const SizedBox(height: 7),
          Wrap(
            spacing: 12,
            runSpacing: 5,
            children: [
              Text(
                'Status received ${model.age(model.statusReceivedAt).toLowerCase()}',
                style: const TextStyle(color: _muted, fontSize: 11),
              ),
              const Text(
                'Illustrative schematic',
                style: TextStyle(color: _muted, fontSize: 11),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _telemetrySection() {
    final data = model.telemetry;
    final fresh = model.telemetryFresh;
    final sampleLabel = data == null
        ? 'Awaiting sample'
        : !fresh
        ? 'Sample unavailable'
        : data.fallen
        ? 'Fall detected'
        : 'No fall reported';
    final sampleColor = !fresh
        ? _amber
        : data!.fallen
        ? _red
        : _mint;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        _section('01', 'Robot telemetry', Icons.sensors_rounded),
        const SizedBox(height: 14),
        _panel(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              _badge(sampleLabel, sampleColor),
              const SizedBox(height: 18),
              LayoutBuilder(
                builder: (context, constraints) {
                  final columns =
                      constraints.maxWidth < 260 ||
                          MediaQuery.textScalerOf(context).scale(1) > 1.4
                      ? 1
                      : 3;
                  final width =
                      (constraints.maxWidth - (columns - 1) * 10) / columns;
                  return Wrap(
                    spacing: 10,
                    runSpacing: 12,
                    children: [
                      _metric(
                        'Height',
                        fresh ? data?.height : null,
                        'm',
                        3,
                        width,
                      ),
                      _metric(
                        'Roll',
                        fresh ? data?.roll : null,
                        'rad',
                        3,
                        width,
                      ),
                      _metric(
                        'Pitch',
                        fresh ? data?.pitch : null,
                        'rad',
                        3,
                        width,
                      ),
                    ],
                  );
                },
              ),
              const SizedBox(height: 22),
              const Text(
                'GROUND CONTACT',
                style: TextStyle(
                  color: _muted,
                  fontSize: 10,
                  letterSpacing: 1.5,
                  fontWeight: FontWeight.w700,
                ),
              ),
              const SizedBox(height: 10),
              LayoutBuilder(
                builder: (context, constraints) {
                  final left = _foot(
                    'Left foot',
                    fresh ? data?.leftFoot : null,
                  );
                  final right = _foot(
                    'Right foot',
                    fresh ? data?.rightFoot : null,
                  );
                  return MediaQuery.textScalerOf(context).scale(1) > 1.4 ||
                          constraints.maxWidth < 260
                      ? Column(
                          children: [left, const SizedBox(height: 10), right],
                        )
                      : Row(
                          children: [
                            Expanded(child: left),
                            const SizedBox(width: 10),
                            Expanded(child: right),
                          ],
                        );
                },
              ),
              const SizedBox(height: 16),
              if (fresh && data!.fallen)
                Padding(
                  padding: const EdgeInsets.only(bottom: 12),
                  child: _notice(
                    'The latest sample reports a fall. Press Stop, then use STAND to reset the MuJoCo simulation.',
                    _red,
                  ),
                ),
              if (model.telemetryError != null)
                Padding(
                  padding: const EdgeInsets.only(bottom: 12),
                  child: _notice(
                    'Telemetry unavailable. ${model.telemetryError}',
                    _amber,
                  ),
                ),
              Text(
                'API sample received ${model.age(model.telemetryReceivedAt).toLowerCase()}',
                style: const TextStyle(color: _muted, fontSize: 11),
              ),
              const SizedBox(height: 6),
              const Text(
                'Source timestamps are unavailable. Values may come from a saved simulator sample.',
                style: TextStyle(color: _muted, fontSize: 11, height: 1.5),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _controlsSection() => Column(
    crossAxisAlignment: CrossAxisAlignment.start,
    children: [
      _section('02', 'Motion control', Icons.gamepad_outlined),
      const SizedBox(height: 14),
      _panel(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Walk pace / speed',
              style: TextStyle(fontWeight: FontWeight.w600, fontSize: 16),
            ),
            const SizedBox(height: 10),
            Text(
              '${model.requestedSpeed.toStringAsFixed(2)}',
              style: const TextStyle(
                fontSize: 32,
                fontWeight: FontWeight.w700,
                letterSpacing: -1,
              ),
            ),
            Slider(
              key: const Key('speed-slider'),
              value: model.requestedSpeed,
              min: 0,
              max: 1,
              divisions: 20,
              label: '${model.requestedSpeed.toStringAsFixed(2)}',
              onChanged: model.busy || model.status?.status == 'STOPPING'
                  ? null : model.setSpeed,
              onChangeEnd: (_) => unawaited(model.applySpeed()),
            ),
            const Row(
              children: [
                Expanded(
                  child: Text(
                    '0.00',
                    style: TextStyle(color: _muted, fontSize: 11),
                  ),
                ),
                Expanded(
                  child: Text(
                    '1.00',
                    textAlign: TextAlign.right,
                    style: TextStyle(color: _muted, fontSize: 11),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 12),
            Text(
              'Backend request: ${model.statusFresh ? '${model.status!.speed.toStringAsFixed(2)}' : '—'}',
              style: const TextStyle(color: _muted, fontSize: 12),
            ),
            const SizedBox(height: 6),
            const Text(
              'RL mode: requested speed in m/s. Release the slider while walking to apply a change. Zero or Stop slows the robot and closes the walking simulation. Scripted mode uses a pace value from 0 to 1.',
              style: TextStyle(color: _muted, fontSize: 12, height: 1.5),
            ),
            if (model.telemetryFresh && model.telemetry?.measuredSpeed != null)
              Padding(
                padding: const EdgeInsets.only(top: 10),
                child: Text(
                  'Applied command: ${model.telemetry!.commandSpeed?.toStringAsFixed(2) ?? "—"} m/s'
                  '  •  Measured: ${model.telemetry!.measuredSpeed!.toStringAsFixed(2)} m/s',
                  style: const TextStyle(color: _mint, fontSize: 12),
                ),
              ),
            const SizedBox(height: 20),
            SizedBox(
              width: double.infinity,
              child: FilledButton.icon(
                key: const Key('walk-button'),
                onPressed: model.canWalk
                    ? () => unawaited(model.sendAction('WALK'))
                    : null,
                icon: const Icon(Icons.directions_walk_rounded),
                label: Text(
                  model.pendingAction == 'WALK'
                      ? 'Sending walk…'
                      : model.statusFresh && model.status?.status == 'WALKING'
                      ? 'Walking controller active'
                      : model.status?.status == 'STARTING'
                      ? 'Loading walking controller…'
                      : 'Start walking',
                ),
              ),
            ),
            const SizedBox(height: 12),
            const Text(
              'Wait for STOPPED before choosing another motion. The RL walking window closes after deceleration.',
              style: TextStyle(color: _muted, fontSize: 11, height: 1.5),
            ),
            if (model.statusFresh && model.status?.message != null &&
                const {'STARTING', 'STOPPING', 'ERROR', 'FALLEN'}.contains(model.status?.status))
              Padding(
                padding: const EdgeInsets.only(top: 12),
                child: _notice(
                  model.status!.message!,
                  const {'ERROR', 'FALLEN'}.contains(model.status?.status) ? _red : _mint,
                ),
              ),
            if (model.commandMessage != null)
              Padding(
                padding: const EdgeInsets.only(top: 16),
                child: Semantics(
                  liveRegion: true,
                  child: _notice(
                    model.commandMessage!,
                    model.commandFailed ? _red : _mint,
                  ),
                ),
              ),
            const SizedBox(height: 12),
            ExpansionTile(
              tilePadding: EdgeInsets.zero,
              childrenPadding: const EdgeInsets.only(bottom: 8),
              title: const Text(
                'MotionCore demo actions',
                style: TextStyle(fontSize: 14),
              ),
              subtitle: const Text(
                'Live MuJoCo motions',
                style: TextStyle(color: _muted, fontSize: 11),
              ),
              children: [
                const Text(
                  'Stand, Wave and Salute use the original robot window. Stop RL walking before switching to these actions.',
                  style: TextStyle(color: _muted, fontSize: 12, height: 1.5),
                ),
                const SizedBox(height: 12),
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: [
                    for (final action in ['STAND', 'WAVE', 'SALUTE'])
                      OutlinedButton(
                        onPressed:
                            model.canDemoAction
                            ? () => unawaited(model.sendAction(action))
                            : null,
                        child: Text(action),
                      ),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    ],
  );

  Widget _stopBar() => Container(
    decoration: const BoxDecoration(
      color: Color(0xFF111B28),
      border: Border(top: BorderSide(color: Color(0xFF293646))),
    ),
    child: SafeArea(
      top: false,
      child: Padding(
        padding: const EdgeInsets.fromLTRB(20, 12, 20, 12),
        child: Center(
          heightFactor: 1,
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 1000),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                FilledButton.icon(
                  key: const Key('stop-button'),
                  style: FilledButton.styleFrom(
                    backgroundColor: const Color(0xFFEF747D),
                    foregroundColor: const Color(0xFF240E16),
                  ),
                  onPressed: model.stopping
                      ? null
                      : () => unawaited(model.stop()),
                  icon: const Icon(Icons.stop_rounded, size: 25),
                  label: Text(
                    model.stopping ? 'Sending stop…' : 'Stop controller',
                  ),
                ),
                if (model.commandMessage != null && model.commandFailed)
                  Padding(
                    padding: const EdgeInsets.only(top: 6),
                    child: Text(
                      model.commandMessage!,
                      maxLines: 3,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(color: _red, fontSize: 11),
                    ),
                  ),
              ],
            ),
          ),
        ),
      ),
    ),
  );

  Widget _panel({required Widget child}) => Container(
    width: double.infinity,
    padding: const EdgeInsets.all(20),
    decoration: BoxDecoration(
      color: const Color(0xFF131E2C),
      borderRadius: BorderRadius.circular(24),
      border: Border.all(color: const Color(0xFF263344)),
    ),
    child: Material(type: MaterialType.transparency, child: child),
  );

  Widget _badge(String text, Color color) => Container(
    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
    decoration: BoxDecoration(
      color: color.withValues(alpha: 0.1),
      borderRadius: BorderRadius.circular(8),
    ),
    child: Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(Icons.circle, size: 6, color: color),
        const SizedBox(width: 7),
        Flexible(
          child: Text(
            text,
            style: TextStyle(
              color: color,
              fontSize: 11,
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
      ],
    ),
  );

  Widget _detail(String label, String value) => Column(
    crossAxisAlignment: CrossAxisAlignment.start,
    children: [
      Text(
        label,
        style: const TextStyle(color: _muted, fontSize: 9, letterSpacing: 1.3),
      ),
      const SizedBox(height: 4),
      Text(
        value,
        style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w600),
      ),
    ],
  );

  Widget _section(String number, String title, IconData icon) => Row(
    children: [
      Text(
        number,
        style: const TextStyle(
          color: _mint,
          fontSize: 12,
          fontWeight: FontWeight.w700,
        ),
      ),
      const SizedBox(width: 10),
      Expanded(
        child: Text(
          title,
          style: const TextStyle(
            fontSize: 19,
            fontWeight: FontWeight.w600,
            letterSpacing: -0.4,
          ),
        ),
      ),
      Icon(icon, size: 20, color: _muted),
    ],
  );

  Widget _metric(
    String label,
    double? value,
    String unit,
    int precision,
    double width,
  ) => SizedBox(
    width: width,
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: const TextStyle(color: _muted, fontSize: 12)),
        const SizedBox(height: 7),
        FittedBox(
          fit: BoxFit.scaleDown,
          alignment: Alignment.centerLeft,
          child: Text(
            value?.toStringAsFixed(precision) ?? '—',
            style: const TextStyle(
              fontSize: 26,
              fontWeight: FontWeight.w600,
              letterSpacing: -0.8,
            ),
          ),
        ),
        const SizedBox(height: 3),
        Text(unit, style: const TextStyle(color: _muted, fontSize: 11)),
      ],
    ),
  );

  Widget _foot(String label, bool? contact) => Container(
    width: double.infinity,
    padding: const EdgeInsets.all(12),
    decoration: BoxDecoration(
      color: contact == true
          ? _mint.withValues(alpha: 0.07)
          : const Color(0xFF0E1824),
      borderRadius: BorderRadius.circular(14),
      border: Border.all(
        color: contact == true
            ? _mint.withValues(alpha: 0.3)
            : const Color(0xFF293747),
      ),
    ),
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Icon(
          Icons.sensors_rounded,
          size: 20,
          color: contact == true ? _mint : _muted,
        ),
        const SizedBox(height: 10),
        Text(label, style: const TextStyle(fontSize: 12)),
        const SizedBox(height: 4),
        Text(
          contact == null
              ? 'UNKNOWN'
              : contact
              ? 'CONTACT'
              : 'AIR',
          style: TextStyle(
            color: contact == true ? _mint : _muted,
            fontSize: 11,
            fontWeight: FontWeight.w700,
            letterSpacing: 0.5,
          ),
        ),
      ],
    ),
  );

  Widget _notice(String text, Color color) => Container(
    width: double.infinity,
    padding: const EdgeInsets.all(12),
    decoration: BoxDecoration(
      color: color.withValues(alpha: 0.08),
      borderRadius: BorderRadius.circular(12),
    ),
    child: Text(
      text,
      style: TextStyle(color: color, fontSize: 12, height: 1.5),
    ),
  );
}
