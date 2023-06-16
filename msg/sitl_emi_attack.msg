uint64 timestamp	# time since system start (microseconds)
int32 attack_trigger	# attack trigger signal
int32 defense_trigger   # defense trigger signal
int32 attack_log_trigger

float32 attack_frequency_g
float32 attack_frequency_a
float32 attack_frequency_m

float32 attack_amplitude_g
float32 attack_amplitude_a
float32 attack_amplitude_m

float32 attack_timediff

float32 compromised_gyro0
float32 compromised_gyro1
float32 compromised_gyro2

float32 benign_gyro0
float32 benign_gyro1
float32 benign_gyro2

float32 compromised_accel0
float32 compromised_accel1
float32 compromised_accel2

float32 benign_accel0
float32 benign_accel1
float32 benign_accel2

float32 compromised_mag0
float32 compromised_mag1
float32 compromised_mag2

float32 benign_mag0
float32 benign_mag1
float32 benign_mag2

int32 defense_window
int32 defense_offset
int32 defense_time
