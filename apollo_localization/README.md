# Localization

## Introduction
  This module provides localization service. It has implemented a RTK based localization which incorporates GPS and IMU information.

## Input
  In the provided RTK localization method, there are two inputs:
  * GPS The Global Position System.
  * IMU Inertial Measurement Unit.

## Output
  * An object instance defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`.

## Add localization implementation
  Currently the RTK based localization is implemented in class `RTKLocalization`. If a new localization method need to be implemented with a name such as `FooLocalization`, you can follow the following steps:

  1. In `proto/localization_config.proto`, add `FOO` in the `LocalizationType` enum type.

  1. Go to the `modules/localization` directory, and create a `foo` directory. In the `foo` directory, implement class `FooLocalization` following the `RTKLocalization` class in the `rtk` directory. `FooLocalization` has to be a subclass of `LocalizationBase`. Also create file foo/BUILD following file `rtk/BUILD`.

  1. You need to register `FooLocalizatoin` class in function `Localization::RegisterLocalizationMethods()`, which is located in cpp file `localization.cc`. You can register by inserting the following code at the end of the function:
  
  ```
  localization_factory_.Register(LocalizationConfing::FOO, []()->LocalizationBase* { return new FooLocalization(); });
  ```
  
  Make sure your code can be compiled by including the header files that defines class `FooLocalization`.

  1. Now you can go back to the `apollo` root directory and build your code with command `bash apollo.sh build`.
