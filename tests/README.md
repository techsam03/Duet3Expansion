# Closed-loop controller helper tests

`ClosedLoopControllerTests.cpp` is a small host test for scalar/pair gain-mode
switching, the commanded move/standstill transition, the separate Coulomb
feedforward transition and split-I anti-windup.
It has no target-hardware dependencies and can be built with a C++17 compiler.

For example, with MSVC:

```
cl /std:c++17 /W4 /WX /EHsc tests\ClosedLoopControllerTests.cpp
```
