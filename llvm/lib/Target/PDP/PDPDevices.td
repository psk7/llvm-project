//===---------------------------------------------------------------------===//
// PDP Device Definitions
//===---------------------------------------------------------------------===//

// A feature set aggregates features, grouping them. We don't want to create a
// new member in PDPSubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
    : SubtargetFeature<name, "HasFeatureSet"#NAME, "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, list<SubtargetFeature> i>
    : FeatureSet<
          name, !strconcat("The device is a part of the ", name, " family"), i>;

def FeatureIMUL : SubtargetFeature<"imul", "HasIMUL", "true",
                                   "The device has random access memory">;

class ELFArch<string name> : SubtargetFeature<"", "ELFArch", !strconcat("ELF::", name), "">;

// ELF e_flags architecture values
def ELFArchVM1 : ELFArch<"EF_PDP_ARCH_VM1">;


//===---------------------------------------------------------------------===//
// PDP Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single AVR
// device should have.
def FamilyVM1 : Family<"VM1", []>;

/*def FeatureSetSpecial
    : FeatureSet<"special",
                 "Enable use of the entire instruction "
                 "set - used for debugging",
                 [
                   FeatureSRAM, FeatureJMPCALL, FeatureIJMPCALL,
                   FeatureEIJMPCALL, FeatureADDSUBIW, FeatureMOVW, FeatureLPM,
                   FeatureLPMX, FeatureELPM, FeatureELPMX, FeatureSPM,
                   FeatureSPMX, FeatureDES, FeatureRMW, FeatureMultiplication,
                   FeatureBREAK, FeatureMMR
                 ]>;*/

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
    : Processor<Name, NoItineraries, !listconcat([Fam, Arch], ExtraFeatures)>;

// Generic CPUs
//def : Device<"VM1", FamilyVM1, ELFArchVM1>;

// Specific CPUs
def : Device<"vm1a", FamilyVM1, ELFArchVM1, []>;
def : Device<"vm1g", FamilyVM1, ELFArchVM1, []>;
