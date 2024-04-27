import 'dart:developer';
import 'dart:io';
import 'dart:async';
import 'dart:typed_data';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

class CameraPage extends StatefulWidget {
  final BluetoothDevice device;
  const CameraPage({Key? key, required this.device}) : super(key: key);

  @override
  State<CameraPage> createState() => _CameraPageState();
}

class _Message {
  int whom;
  String text;

  _Message(this.whom, this.text);
}

class _CameraPageState extends State<CameraPage> with WidgetsBindingObserver {
  CameraController? _camController;
  int _camFrameRotation = 0;
  double _camFrameToScreenScale = 0;
  BluetoothConnection? connection;

  List<_Message> messages = [];
  String _messageBuffer = '';

  final TextEditingController textEditingController = TextEditingController();
  final ScrollController listScrollController = ScrollController();

  bool isConnecting = true;
  bool get isConnected => connection!.isConnected;

  bool isDisconnecting = false;
  Timer? timer;
  int _lastRun = 0;
  bool _detectionInProgress = false;
  bool first_time = true;
  var zooming = 0.0;
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    initCamera();
    BluetoothConnection.toAddress(widget.device.address).then((_connection) {
      print('Connected to the device');
      connection = _connection;
      setState(() {
        isConnecting = false;
        isDisconnecting = false;
      });

      connection!.input!.listen(_onDataReceived).onDone(() {
        if (isDisconnecting) {
          print('Disconnecting locally!');
        } else {
          print('Disconnected remotely!');
        }
        if (mounted) {
          setState(() {});
        }
      });
    }).catchError((error) {
      print('Cannot connect, exception occured');
      print(error);
    });
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    final CameraController? cameraController = _camController;
    // App state changed before we got the chance to initialize.
    if (cameraController == null || !cameraController.value.isInitialized) {
      return;
    }

    if (state == AppLifecycleState.inactive) {
      cameraController.dispose();
    } else if (state == AppLifecycleState.resumed) {
      initCamera();
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _camController?.dispose();
    if (isConnected) {
      isDisconnecting = true;
      connection!.dispose();
      // connection = null;
    }
    print("error in cccccccccccccc");
    super.dispose();
  }

  Future<void> initCamera() async {
    final cameras = await availableCameras();
    var idx =
        cameras.indexWhere((c) => c.lensDirection == CameraLensDirection.back);
    if (idx < 0) {
      log("No Back camera found - weird");
      return;
    }

    var desc = cameras[idx];
    _camFrameRotation = Platform.isAndroid ? desc.sensorOrientation : 0;
    _camController = CameraController(
      desc,
      ResolutionPreset.high, // 720p
      enableAudio: false,
      imageFormatGroup: Platform.isAndroid
          ? ImageFormatGroup.yuv420
          : ImageFormatGroup.bgra8888,
    );

    try {
      await _camController!.initialize();
    } catch (e) {
      log("Error initializing camera, error: ${e.toString()}");
    }

    if (mounted) {
      setState(() {});
    }
  }

  void take_photo_first() async {
    // var image = await _camController!.takePicture();
    _sendMessage("0");
  }

  void _sendMessage(String text) async {
    text = text.trim();
    textEditingController.clear();
    try {
      connection!.output.add(Uint8List.fromList(utf8.encode(text)));
      await connection!.output.allSent;
    } catch (e) {}
  }

  void _onDataReceived(Uint8List data) {
    // Allocate buffer for parsed data
    int backspacesCounter = 0;
    for (var byte in data) {
      if (byte == 8 || byte == 127) {
        backspacesCounter++;
      }
    }
    Uint8List buffer = Uint8List(data.length - backspacesCounter);
    int bufferIndex = buffer.length;

    // Apply backspace control character
    backspacesCounter = 0;
    for (int i = data.length - 1; i >= 0; i--) {
      if (data[i] == 8 || data[i] == 127) {
        backspacesCounter++;
      } else {
        if (backspacesCounter > 0) {
          backspacesCounter--;
        } else {
          buffer[--bufferIndex] = data[i];
        }
      }
    }

    // Create message if there is new line character
    String dataString = String.fromCharCodes(buffer);
    int index = buffer.indexOf(13);
    if (~index != 0) {
      setState(() {
        messages.add(
          _Message(
            1,
            backspacesCounter > 0
                ? _messageBuffer.substring(
                    0, _messageBuffer.length - backspacesCounter)
                : _messageBuffer + dataString.substring(0, index),
          ),
        );
        _messageBuffer = dataString.substring(index);
      });
    } else {
      _messageBuffer = (backspacesCounter > 0
          ? _messageBuffer.substring(
              0, _messageBuffer.length - backspacesCounter)
          : _messageBuffer + dataString);
    }
  }

  void _screenShot() async {
    // await _screenshotController!
    //     .capture(delay: const Duration(milliseconds: 10))
    //     .then((Uint8List? image) async {
    //   print(
    //       "====================================image====================$image");
    //   XFile xFile = XFile.fromData(image as Uint8List);
    //   await GallerySaver.saveImage(xFile.path);
    // });
    // var image = await _camController!.takePicture();
    // await GallerySaver.saveImage(image.path);
    // print("======================take Screen Shot=====================");
  }

  @override
  Widget build(BuildContext context) {
    if (_camController == null) {
      return const Center(
        child: CircularProgressIndicator(),
      );
    }
    return Column(
      children: [
        Expanded(
          child: AspectRatio(
            aspectRatio: _camController!.value.aspectRatio,
            child: CameraPreview(_camController!),
          ),
        ),
        Row(
          children: [
            FloatingActionButton(
              heroTag: "btn1",
              // Provide an onPressed callback.
              onPressed: () => take_photo_first(),
              child: const Icon(Icons.camera_alt),
            )
          ],
        )
      ],
    );
  }
}
