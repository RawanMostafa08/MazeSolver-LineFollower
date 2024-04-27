import 'dart:math';

import 'package:bar2banzzen/constant.dart';
import 'package:bar2banzzen/pages/splash_screen.dart';
import 'package:flutter/material.dart';
import 'package:native_opencv/native_opencv.dart';

void main() {
  runApp(Bar2banzzen());
}

class Bar2banzzen extends StatelessWidget {
  Bar2banzzen({super.key});

  NativeOpencv nativeOpencv = NativeOpencv();
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme:
          ThemeData.dark().copyWith(scaffoldBackgroundColor: Colors.deepPurple),
      // routes: {
      //   ksplash: (context) => const SplashScreen(),
      // },
      // initialRoute: ksplash,
      home: Container(
        child: ElevatedButton(
          onPressed: () async {
            log('----------------${nativeOpencv.cvVersion()}' as num);
          },
          child: const Text('Load Model'),
        ),
      ),
    );
  }
}
