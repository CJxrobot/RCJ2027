#pragma once
struct FakeWire {
  void begin() {}
  void setClock(long) {}
};
extern FakeWire Wire;
