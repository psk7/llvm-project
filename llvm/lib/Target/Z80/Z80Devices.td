//===---------------------------------------------------------------------===//
// Z80 Device Definitions
//===---------------------------------------------------------------------===//

class Device<string Name>
  : Processor<Name, NoItineraries, []>;

// Generic MCUs
// Note that several versions of GCC has strange ELF architecture
// settings for backwards compatibility - see `gas/config/tc-avr.c`
// in Z80 binutils. We do not replicate this.
def : Device<"z80">;
