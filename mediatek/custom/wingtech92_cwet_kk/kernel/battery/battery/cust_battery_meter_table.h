#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900	
#define RBAT_PULL_DOWN_R		   27000	
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900	
#define RBAT_PULL_DOWN_R		  100000	
#endif
#define RBAT_PULL_UP_VOLT          1800



// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,67790},
        {-15,53460},
        {-10,42450},
        { -5,33930},
        {  0,27280},
        {  5,22070},
        { 10,17960},
        { 15,14700},
        { 20,12090},
        { 25,10000},
        { 30,8312},
        { 35,6942},
        { 40,5826},
        { 45,4911},
        { 50,4158},
        { 55,3536},
        { 60,3019}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	{0   , 4318},         
	{2   , 4296},         
	{4   , 4277},         
	{5   , 4257},         
	{7   , 4237},         
	{9   , 4213},         
	{11  , 4183},         
	{13  , 4155},         
	{15  , 4128},         
	{16  , 4108},         
	{18  , 4093},         
	{20  , 4076},         
	{22  , 4059},         
	{24  , 4043},         
	{26  , 4027},         
	{27  , 4010},         
	{29  , 3996},         
	{31  , 3983},         
	{33  , 3969},         
	{35  , 3957},         
	{37  , 3943},         
	{38  , 3928},         
	{40  , 3913},         
	{42  , 3897},         
	{44  , 3883},         
	{46  , 3871},         
	{47  , 3861},         
	{49  , 3852},         
	{51  , 3846},         
	{53  , 3836},         
	{55  , 3831},         
	{57  , 3832},         
	{58  , 3821},         
	{60  , 3814},         
	{62  , 3807},         
	{64  , 3803},         
	{66  , 3797},         
	{68  , 3795},         
	{69  , 3790},         
	{71  , 3788},         
	{73  , 3783},         
	{75  , 3780},         
	{77  , 3773},         
	{79  , 3767},         
	{80  , 3757},         
	{82  , 3747},         
	{84  , 3734},         
	{86  , 3723},         
	{88  , 3713},         
	{89  , 3707},         
	{91  , 3701},         
	{93  , 3696},         
	{94  , 3691},         
	{95  , 3688},         
	{96  , 3685},         
	{96  , 3682},         
	{97  , 3676},         
	{97  , 3672},         
	{98  , 3667},         
	{98  , 3663},         
	{98  , 3656},          
  {98  , 3652},
  {99  , 3648},
  {99  , 3644},
  {99  , 3641},
  {99  , 3637},
  {99  , 3634},
  {99  , 3632},
  {99  , 3630},
  {99  , 3628},
	{100 , 3625}, 
	{100 , 3624},
	{100 , 3623},
	{100 , 3621},
	{100 , 3619},
	{100 , 3618}, 
	{100 , 3616}	       
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0   , 4313},         
	{2   , 4281},         
	{3   , 4258},         
	{5   , 4240},         
	{7   , 4221},         
	{8   , 4204},         
	{10  , 4187},         
	{11  , 4172},         
	{13  , 4156},         
	{15  , 4140},         
	{16  , 4124},         
	{18  , 4109},         
	{20  , 4094},         
	{21  , 4081},         
	{23  , 4067},         
	{24  , 4051},         
	{26  , 4035},         
	{28  , 4021},         
	{29  , 4006},         
	{31  , 3992},         
	{33  , 3979},         
	{34  , 3967},         
	{36  , 3955},         
	{37  , 3939},         
	{39  , 3925},         
	{41  , 3910},         
	{42  , 3896},         
	{44  , 3883},         
	{46  , 3871},         
	{47  , 3860},         
	{49  , 3853},         
	{50  , 3846},         
	{52  , 3838},         
	{54  , 3832},         
	{55  , 3826},         
	{57  , 3818},         
	{59  , 3813},         
	{60  , 3807},         
	{62  , 3802},         
	{63  , 3798},         
	{65  , 3793},         
	{67  , 3790},         
	{68  , 3786},         
	{70  , 3783},         
	{72  , 3779},         
	{73  , 3775},         
	{75  , 3772},         
	{76  , 3765},         
	{78  , 3755},         
	{80  , 3744},         
	{81  , 3730},         
	{83  , 3715},         
	{85  , 3705},         
	{86  , 3699},         
	{88  , 3696},         
	{90  , 3694},         
	{91  , 3688},         
	{93  , 3674},         
  {94  , 3636},
	{96  , 3585},         
	{97  , 3550},          
  {98  , 3534},
  {98  , 3523},
  {98  , 3511},
  {99  , 3503},
  {99  , 3497},
  {99  , 3490},
  {99  , 3487},
  {99  , 3483},
  {99  , 3478},
	{100 , 3476}, 
	{100 , 3475},
	{100 , 3472},
	{100 , 3469},
	{100 , 3467},
	{100 , 3466}, 
	{100 , 3462}	       
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0   , 4336},         
	{2   , 4312},         
	{3   , 4292},         
	{5   , 4274},         
	{6   , 4257},         
	{8   , 4242},         
	{9   , 4224},         
	{11  , 4207},         
	{12  , 4191},         
	{14  , 4175},         
	{15  , 4159},         
	{17  , 4142},         
	{18  , 4127},         
	{20  , 4111},         
	{21  , 4097},         
	{23  , 4084},         
	{24  , 4070},         
	{26  , 4055},         
	{27  , 4040},         
	{29  , 4027},         
	{30  , 4013},         
	{32  , 3999},         
	{33  , 3988},         
	{35  , 3977},         
	{36  , 3966},         
	{38  , 3954},         
	{39  , 3941},         
	{41  , 3928},         
	{42  , 3910},         
	{44  , 3894},         
	{45  , 3881},         
	{47  , 3868},         
	{48  , 3858},         
	{50  , 3850},         
	{51  , 3844},         
	{53  , 3836},         
	{54  , 3829},         
	{56  , 3824},         
	{58  , 3817},         
	{59  , 3812},         
	{61  , 3806},         
	{62  , 3802},         
	{64  , 3796},         
	{65  , 3792},         
	{67  , 3787},         
	{68  , 3782},         
	{70  , 3778},         
	{71  , 3770},         
	{73  , 3764},         
	{74  , 3753},         
	{76  , 3746},         
	{77  , 3737},         
	{79  , 3723},         
	{80  , 3712},         
	{82  , 3696},         
	{83  , 3690},         
	{85  , 3688},         
	{86  , 3685},         
	{88  , 3683},         
	{89  , 3676},         
	{91  , 3657},          
  {92  , 3632},
  {94  , 3602},
  {95  , 3561},
  {97  , 3511},
  {98  , 3459},
  {100 , 3407},
  {100 , 3368},
  {100 , 3339},
  {100 , 3321},
	{100 , 3307}, 
	{100 , 3297},
	{100 , 3292},
	{100 , 3286},
	{100 , 3283},
	{100 , 3278},
	{100 , 3278}	       
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0   , 4340},         
	{2   , 4317},         
	{3   , 4297},         
	{5   , 4279},         
	{6   , 4261},         
	{8   , 4246},         
	{9   , 4228},         
	{11  , 4212},         
	{12  , 4195},         
	{14  , 4177},         
	{15  , 4161},         
	{17  , 4145},         
	{18  , 4130},         
	{20  , 4115},         
	{21  , 4099},         
	{23  , 4084},         
	{24  , 4070},         
	{26  , 4055},         
	{27  , 4042},         
	{29  , 4030},         
	{31  , 4014},         
	{32  , 4004},         
	{34  , 3991},         
	{35  , 3979},         
	{37  , 3968},         
	{38  , 3957},         
	{40  , 3945},         
	{41  , 3932},         
	{43  , 3915},         
	{44  , 3896},         
	{46  , 3880},         
	{47  , 3869},         
	{49  , 3861},         
	{50  , 3851},         
	{52  , 3843},         
	{53  , 3836},         
	{55  , 3830},         
	{56  , 3824},         
	{58  , 3817},         
	{59  , 3811},         
	{61  , 3806},         
	{63  , 3800},         
	{64  , 3795},         
	{66  , 3791},         
	{67  , 3782},         
	{69  , 3768},         
	{70  , 3761},         
	{72  , 3752},         
	{73  , 3745},         
	{75  , 3736},         
	{76  , 3729},         
	{78  , 3720},         
	{79  , 3705},         
	{81  , 3692},         
	{82  , 3679},         
	{84  , 3676},         
	{85  , 3672},         
	{87  , 3670},         
	{88  , 3669},         
	{90  , 3657},         
	{91  , 3637},          
  {93  , 3610},
  {95  , 3576},
  {96  , 3533},
  {98  , 3486},
  {99  , 3434},
  {100 , 3377},
  {100 , 3319},
  {100 , 3287},
  {100 , 3273},
	{100 , 3266}, 
	{100 , 3262},
	{100 , 3259},
	{100 , 3258},
	{100 , 3254},
	{100 , 3253},
	{100 , 3254}	       
};           

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{473  , 4318},         
	{473  , 4296},         
	{478  , 4277},         
	{488  , 4257},         
	{503  , 4237},         
	{523  , 4213},         
	{555  , 4183},         
	{603  , 4155},         
	{650  , 4128},         
	{683  , 4108},         
	{705  , 4093},         
	{728  , 4076},         
	{738  , 4059},         
	{748  , 4043},         
	{748  , 4027},         
	{745  , 4010},         
	{748  , 3996},         
	{755  , 3983},         
	{755  , 3969},         
	{760  , 3957},         
	{768  , 3943},         
	{768  , 3928},         
	{770  , 3913},         
	{768  , 3897},         
	{768  , 3883},         
	{768  , 3871},         
	{775  , 3861},         
	{783  , 3852},         
	{793  , 3846},         
	{795  , 3836},         
	{808  , 3831},         
	{835  , 3832},         
	{833  , 3821},         
	{845  , 3814},         
	{855  , 3807},         
	{868  , 3803},         
	{878  , 3797},         
	{893  , 3795},         
	{905  , 3790},         
	{923  , 3788},         
	{935  , 3783},         
	{953  , 3780},         
	{968  , 3773},         
	{988  , 3767},         
	{1003 , 3757},         
	{1030 , 3747},         
	{1055 , 3734},         
	{1085 , 3723},         
	{1110 , 3713},         
	{1143 , 3707},         
	{1175 , 3701},         
	{1223 , 3696},         
	{1228 , 3691},         
	{1220 , 3688},         
	{1215 , 3685},         
	{1205 , 3682},         
	{1193 , 3676},         
	{1180 , 3672},         
	{1170 , 3667},         
	{1160 , 3663},         
	{1140 , 3656},          
  {1133 , 3652},
  {1123 , 3648},
  {1110 , 3644},
  {1105 , 3641},
  {1095 , 3637},
  {1090 , 3634},
  {1083 , 3632},
  {1075 , 3630},
  {1078 , 3628},
	{1065 , 3625}, 
	{1063 , 3624},
	{1060 , 3623},
	{1055 , 3621},
	{1048 , 3619},
	{1048 , 3618}, 
	{1043 , 3616}	       
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{335 , 4313},         
	{335 , 4281},         
	{340 , 4258},         
	{350 , 4240},         
	{353 , 4221},         
	{355 , 4204},         
	{360 , 4187},         
	{365 , 4172},         
	{370 , 4156},         
	{380 , 4140},         
	{383 , 4124},         
	{388 , 4109},         
	{390 , 4094},         
	{400 , 4081},         
	{420 , 4067},         
	{425 , 4051},         
	{433 , 4035},         
	{435 , 4021},         
	{430 , 4006},         
	{433 , 3992},         
	{430 , 3979},         
	{430 , 3967},         
	{430 , 3955},         
	{423 , 3939},         
	{423 , 3925},         
	{410 , 3910},         
	{408 , 3896},         
	{400 , 3883},         
	{398 , 3871},         
	{393 , 3860},         
	{400 , 3853},         
	{408 , 3846},         
	{413 , 3838},         
	{418 , 3832},         
	{420 , 3826},         
	{420 , 3818},         
	{430 , 3813},         
	{433 , 3807},         
	{440 , 3802},         
	{440 , 3798},         
	{448 , 3793},         
	{453 , 3790},         
	{465 , 3786},         
	{473 , 3783},         
	{478 , 3779},         
	{488 , 3775},         
	{500 , 3772},         
	{513 , 3765},         
	{520 , 3755},         
	{538 , 3744},         
	{553 , 3730},         
	{573 , 3715},         
	{590 , 3705},         
	{610 , 3699},         
	{638 , 3696},         
	{670 , 3694},         
	{710 , 3688},         
	{753 , 3674},         
	{793 , 3636},         
	{855 , 3585},         
	{875 , 3550},          
  {838 , 3534},
  {810 , 3523},
  {780 , 3511},
  {763 , 3503},
  {745 , 3497},
  {730 , 3490},
  {725 , 3487},
  {713 , 3483},
  {698 , 3478},
	{690 , 3476}, 
	{690 , 3475},
	{683 , 3472},
	{675 , 3469},
	{673 , 3467},
	{668 , 3466}, 
	{658 , 3462}	       
};     

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{140  , 4336},         
	{140  , 4312},         
	{133  , 4292},         
	{138  , 4274},         
	{140  , 4257},         
	{150  , 4242},         
	{145  , 4224},         
	{143  , 4207},         
	{148  , 4191},         
	{153  , 4175},         
	{150  , 4159},         
	{153  , 4142},         
	{150  , 4127},         
	{150  , 4111},         
	{155  , 4097},         
	{160  , 4084},         
	{163  , 4070},         
	{163  , 4055},         
	{165  , 4040},         
	{170  , 4027},         
	{173  , 4013},         
	{178  , 3999},         
	{183  , 3988},         
	{185  , 3977},         
	{190  , 3966},         
	{195  , 3954},         
	{190  , 3941},         
	{190  , 3928},         
	{178  , 3910},         
	{168  , 3894},         
	{158  , 3881},         
	{153  , 3868},         
	{148  , 3858},         
	{148  , 3850},         
	{153  , 3844},         
	{153  , 3836},         
	{150  , 3829},         
	{158  , 3824},         
	{153  , 3817},         
	{160  , 3812},         
	{158  , 3806},         
	{163  , 3802},         
	{160  , 3796},         
	{165  , 3792},         
	{163  , 3787},         
	{165  , 3782},         
	{163  , 3778},         
	{160  , 3770},         
	{155  , 3764},         
	{150  , 3753},         
	{153  , 3746},         
	{153  , 3737},         
	{153  , 3723},         
	{165  , 3712},         
	{165  , 3696},         
	{160  , 3690},         
	{163  , 3688},         
	{168  , 3685},         
	{180  , 3683},         
	{188  , 3676},         
	{220  , 3657},          
  {255  , 3632},
  {270  , 3602},
  {285  , 3561},
  {315  , 3511},
  {363  , 3459},
  {435  , 3407},
  {423  , 3368},
  {350  , 3339},
  {305  , 3321},
	{273  , 3307}, 
	{245  , 3297},
	{230  , 3292},
	{215  , 3286},
	{208  , 3283},
	{200  , 3278}, 
	{198  , 3278}	       
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{113  , 4340},         
	{113  , 4317},         
	{118  , 4297},         
	{115  , 4279},         
	{115  , 4261},         
	{125  , 4246},         
	{123  , 4228},         
	{120  , 4212},         
	{123  , 4195},         
	{123  , 4177},         
	{123  , 4161},         
	{120  , 4145},         
	{123  , 4130},         
	{128  , 4115},         
	{128  , 4099},         
	{128  , 4084},         
	{128  , 4070},         
	{123  , 4055},         
	{133  , 4042},         
	{133  , 4030},         
	{128  , 4014},         
	{140  , 4004},         
	{140  , 3991},         
	{143  , 3979},         
	{148  , 3968},         
	{153  , 3957},         
	{158  , 3945},         
	{160  , 3932},         
	{155  , 3915},         
	{140  , 3896},         
	{128  , 3880},         
	{128  , 3869},         
	{130  , 3861},         
	{125  , 3851},         
	{125  , 3843},         
	{128  , 3836},         
	{128  , 3830},         
	{130  , 3824},         
	{133  , 3817},         
	{133  , 3811},         
	{135  , 3806},         
	{138  , 3800},         
	{140  , 3795},         
	{145  , 3791},         
	{140  , 3782},         
	{128  , 3768},         
	{130  , 3761},         
	{125  , 3752},         
	{128  , 3745},         
	{128  , 3736},         
	{133  , 3729},         
	{133  , 3720},         
	{130  , 3705},         
	{133  , 3692},         
	{133  , 3679},         
	{130  , 3676},         
	{130  , 3672},         
	{135  , 3670},         
	{140  , 3669},         
	{143  , 3657},         
	{195  , 3637},          
  {193  , 3610},
  {205  , 3576},
  {210  , 3533},
  {228  , 3486},
  {240  , 3434},
  {263  , 3377},
  {298  , 3319},
  {223  , 3287},
  {183  , 3273},
	{165  , 3266}, 
	{155  , 3262},
	{150  , 3259},
	{148  , 3258},
	{143  , 3254},
	{138  , 3253}, 
	{135  , 3254}	       
}; 

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

