import 'package:animated_splash_screen/animated_splash_screen.dart';
import 'package:bar2banzzen/pages/home.dart';
import 'package:flutter/material.dart';
import 'package:lottie/lottie.dart';

class SplashScreen extends StatelessWidget {
  const SplashScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return AnimatedSplashScreen(
      backgroundColor: Colors.deepPurple,
      splash: Center(
        child: LottieBuilder.asset("assets/car.json"),
      ),
      nextScreen: const HomePage(),
      splashIconSize: 500,
    );
  }
}
