#include <cstdio>

#include "parity_emit.h"

int parityMain(int argc, char** argv);

int main(int argc, char** argv) {
  const int rc = parityMain(argc, argv);
  std::fflush(parity::out());
  return rc;
}
