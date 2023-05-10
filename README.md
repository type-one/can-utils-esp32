C++ DBC Parser and CAN Transcoder
=================================
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Introduction
------------

This repository is motivated by a lack of suitable open-source DBC parsers in C++.

We provide a fast, lightweight, and fully-featured DBC parser.

The project also contains a CAN transcoder, showing how the parser can be used for [IOT telemetry](https://iotatlas.net/en/patterns/telemetry/).

Features
--------
* [DBC parser](dbc/README.md)
    * A complete and customizable DBC parser written in C++. Full DBC syntax support for all keywords.
* [Vehicle-To-Cloud Transcoder](v2c/README.md)
    * Edge-computing telemetric component that groups, filters, and aggregates CAN signals. Can drastically reduce the amount of data sent from the device over the network.
Uses the DBC parser to read and define the CAN network.

How to Build
------------

#### 1. Fetch Boost

* Download [Boost](https://www.boost.org/users/download/) and move it to your include path

The project requires only headers from Boost, so no libraries need to be built.

#### 2. Build

You can compile the example as follows:

```sh
$ g++ -std=c++20 example\example.cpp dbc\dbc_parser.cpp v2c\v2c_transcoder.cpp -I . -o can_example
```

`can-utils` has been tested with Clang, GCC and MSVC on Windows and Linux. It requires C++20.

Usage
-----

### Example
- [Full source here](example/example.cpp)

Build, then run without any command line arguments:

```sh
$ ./can_example
```

The example program parses [example.dbc](example/example.dbc), generates millions of random frames, aggregates them with `v2c_transcoder`, and prints the decoded raw signals to the console.

Example output:

```py
New frame_packet (from 2121812 frames):
 can_frame at t: 1683709842.116000s, can_id: 4
  SOCavg: 574
 can_frame at t: 1683709842.116000s, can_id: 6
  RawBattCurrent: 10914
  SmoothBattCurrent: 10921
  BattVoltage: 32760
 can_frame at t: 1683709842.516000s, can_id: 2
  GPSAccuracy: 118
  GPSLongitude: -106019721
  GPSLatitude: 26758102
 can_frame at t: 1683709842.516000s, can_id: 3
  GPSAltitude: -8084
 can_frame at t: 1683709842.516000s, can_id: 5
  GPSSpeed: 2160
 can_frame at t: 1683709842.516000s, can_id: 7
  PowerState: 2
 can_frame at t: 1683709842.616000s, can_id: 4
  SOCavg: 163
 can_frame at t: 1683709842.616000s, can_id: 6
  RawBattCurrent: -27877
  SmoothBattCurrent: -27827
  BattVoltage: 32731
  ...
```

The signal values are raw decoded bytes, not scaled by the signal's factor or offset.

___

### DBC Parser

- [Full documentation here.](dbc/README.md)

The parser can be used as follows:

```cpp
custom_dbc dbc_impl; // custom class that implements your logic and data structures

bool success = can::parse_dbc(dbc_content, std::ref(dbc_impl)); // parses the DBC

// dbc_impl is now populated by the parser and can be used
```

The behavior of the parser is customized by user-defined callbacks invoked when parsing a DBC keyword.

Defining the following callback would print all `BO_` objects (messages) in the DBC, and call `add_message()` on `dbc_impl`:

``` cpp
inline void tag_invoke(
	def_bo_cpo, dbc_impl& this_,
	uint32_t msg_id, std::string msg_name, size_t msg_size, size_t transmitter_ord
) {
	std::cout << "New message '" << msg_name << "' with ID = " << msg_id << std::endl;
	this_.add_message(msg_id, msg_name, msg_size);
}
```

The full list of callback function signatures, with examples, can be found [here](dbc/README.md).

___

### V2C Transcoder

- [Full documentation here](v2c/README.md)

V2C is modeled as a node in the CAN network. It reads CAN frames as input, aggregates their values, and encodes them back into CAN `frame_packets`.

To use it, initialize `v2c_transcoder` and then call its `transcode(t, frame)` method with frames read from the CAN socket.

`transcode()` will periodically return a `frame_packet` containing the aggregated `can_frames`, ready to be sent over the network.

```cpp
can::v2c_transcoder transcoder;
can::parse_dbc(read_file("example/example.dbc"), std::ref(transcoder));

while (true) {
	// read a frame from the CAN socket
	can_frame frame = read_frame();

	auto t = std::chrono::system_clock::now();
	auto fp = transcoder.transcode(t, frame);

	if (fp) {
		// send the frame_packet over the network
		send_frame_packet(fp);
	}
}
```

The transcoder's message groups, aggregation types and sampling/sending windows are customized through the DBC directly:

```py
EV_ V2CTxTime: 0 [0|60000] "ms" 2000 1 DUMMY_NODE_VECTOR1 V2C;

EV_ GPSGroupTxFreq: 0 [0|60000] "ms" 600 11 DUMMY_NODE_VECTOR1 V2C;
EV_ EnergyGroupTxFreq: 0 [0|60000] "ms" 500 13 DUMMY_NODE_VECTOR1 V2C;

BA_ "AggType" SG_  7 PowerState "LAST";
BA_ "AggType" SG_  4 SOCavg "LAST";
BA_ "AggType" SG_  6 RawBattCurrent "AVG";
BA_ "AggType" SG_  6 SmoothBattCurrent "AVG";
```

A more in-depth explanation can be found [here](v2c/README.md).

License
-------

Copyright (c) 2001-2023 Mireo, EU

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Contact us
---------- 

<p align="center">
<a href="https://www.mireo.com/spacetime"><img height="200" alt="Mireo" src="https://www.mireo.com/img/assets/mireo-logo.svg"></img></a>
</p>
