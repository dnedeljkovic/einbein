// DACs offsets
static constexpr double dacResolution = 65535;
// DAC mot2
static constexpr double dac0min = -9.557;
static constexpr double dac0max =  9.383;
static constexpr double dac0lsbs = (dac0max - dac0min) / (64000.0 - 2000);
static constexpr double x0min = 2000 - (dac0min + 10.0) / dac0lsbs;
static constexpr double x0max = 64000 - (dac0max - 10.0) / dac0lsbs;
static constexpr double m0 = (x0max - x0min) / (2 * 10.0);
static constexpr double dac0Scale  = 1.0 / m0;
static constexpr double dac0Offset = -(10.0 + x0min / m0);
// DAC res
static constexpr double dac1min = -9.265;
static constexpr double dac1max =  9.520;
static constexpr double dac1lsbs = (dac1max - dac1min) / (64000.0 - 2000);
static constexpr double x1min = 2000 - (dac1min + 10.0) / dac1lsbs;
static constexpr double x1max = 64000 - (dac1max - 10.0) / dac1lsbs;
static constexpr double m1 = (x1max - x1min) / (2 * 10.0);
static constexpr double dac1Scale  = 1.0 / m1;
static constexpr double dac1Offset = -(10.0 + x1min / m1);
//DAC mot1
static constexpr double dac2min = -9.264;
static constexpr double dac2max =  9.557;
static constexpr double dac2lsbs = (dac2max - dac2min) / (64000.0 - 2000);
static constexpr double x2min = 2000 - (dac2min + 10.0) / dac2lsbs;
static constexpr double x2max = 64000 - (dac2max - 10.0) / dac2lsbs;
static constexpr double m2 = (x2max - x2min) / (2 * 10.0);
static constexpr double dac2Scale  = 1.0 / m2;
static constexpr double dac2Offset = -(10.0 + x2min / m2);
//DAC mot3
static constexpr double dac3min = -9.287;
static constexpr double dac3max =  9.547;
static constexpr double dac3lsbs = (dac3max - dac3min) / (64000.0 - 2000);
static constexpr double x3min = 2000 - (dac3min + 10.0) / dac3lsbs;
static constexpr double x3max = 64000 - (dac3max - 10.0) / dac3lsbs;
static constexpr double m3 = (x3max - x3min) / (2 * 10.0);
static constexpr double dac3Scale  = 1.0 / m3;
static constexpr double dac3Offset = -(10.0 + x3min / m3);

//FQD offsets
static constexpr double fqd0scale = 1;
static constexpr double fqd0offset = 1;
static constexpr bool fqd0delta = false;

static constexpr double fqd1scale = 1;
static constexpr double fqd1offset = 1;
static constexpr bool fqd1delta = false;

static constexpr double fqd2scale = 1;
static constexpr double fqd2offset = 1;
static constexpr bool fqd2delta = false;
