import 'package:bar2banzzen/bluetooth/connection.dart';
import 'package:dotted_border/dotted_border.dart';
import 'package:flutter/material.dart';

class HomePage extends StatelessWidget {
  const HomePage({
    super.key,
  });

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text(
          "Bar2 Banzzen",
          style: TextStyle(),
        ),
      ),
      body: SingleChildScrollView(
        child: Column(
          children: [
            SizedBox(
              width: MediaQuery.of(context).size.width * 0.9,
              height: MediaQuery.of(context).size.height * 0.78,
              child: Padding(
                padding: EdgeInsets.symmetric(
                    vertical: MediaQuery.of(context).size.height * 0.05),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Center(
                      child: Text(
                        "Let's Follow The Line",
                        style: TextStyle(
                            fontSize: 25,
                            fontWeight: FontWeight.bold,
                            color: Colors.black),
                      ),
                    ),
                    SizedBox(
                      height: 150,
                    ),
                    Padding(
                      padding: const EdgeInsets.only(left: 40),
                      child: Center(
                        child: GestureDetector(
                          onTap: () {
                            Navigator.push(
                                context,
                                MaterialPageRoute(
                                    builder: (context) =>
                                        const SelectBondedDevicePage()));
                          },
                          child: DottedBorder(
                            borderType: BorderType.RRect,
                            color: Colors.blue,
                            dashPattern: const [6],
                            strokeWidth: 2,
                            padding: const EdgeInsets.symmetric(
                                vertical: 30, horizontal: 40),
                            radius: const Radius.circular(12),
                            child: const Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                Icon(
                                  Icons.add_a_photo_outlined,
                                  color: Colors.blue,
                                  size: 50,
                                ),
                                Padding(
                                  padding: EdgeInsets.only(top: 5.0, left: 3.0),
                                  child: Text(
                                    "Start ",
                                    textAlign: TextAlign.right,
                                    style: TextStyle(
                                        color: Colors.blue,
                                        fontSize: 20,
                                        fontWeight: FontWeight.bold),
                                  ),
                                )
                              ],
                            ),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
