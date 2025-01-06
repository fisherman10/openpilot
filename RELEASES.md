Version 9.8.0  (2024-12-26)
=========================
* **ALERT**: If Assisted Lane Change is enabled, below Assisted Lane Change speed, and turn signal is on, lane keep will be deactivated to allow manual steering control.
* Allow Assisted Lane Change to be cancelled if turn signal is off, or blind spot detected during a lane change
* Smoother steering for stop and go traffic
* Allow lower minimum screen brightness
* Allow multiple features to be set in features package
* Proton bukapilot lane keep follows LKS enable, to use bukapilot lane keep without stock LDW, set LKS to Warn Only and Tactile
* Proton Lane Departure Prevention fix (LKS Auxiliary mode)
* Proton one-touch turn signal of 3.5 seconds when Assisted Lane Change is not active
* Improvements and new tuning for Proton port
* UI improvements
* Code optimisation

Version 9.7.0  (2024-10-07)
=========================
* Smoother steering after resume/lane change with ALC off
* Added "Upload Drive Video via Wi-Fi Only" toggle
* UI improvements
* Fix GPS date
* Lane Departure Warning improvements
* Proton SNG fix
* BYD SNG fix
* Proton LKA follows LKS on/off
* Proton Lane Departure Prevention
* Bypass Honda City HEV low speed steer
* Removed steering limit warning when LKA is off

Version 9.6  (2024-02-06)
=========================
* Planner changes for vehicles with vision only ACC
* NEOS apk OTA enabled
* Logs are now all in UTC format, fixing KommuApp date time
* BYD:
  * Fix understeers on highway
  * Fix auto highbeams aka HMA
  * Fix for steer fault when override
  * Enable separate engage for LKA after ACC is on
  * Reduced occurence of SNG fault
  * LSS settings fix
* Hyundai Ioniq:
  * Change warning behaviour
* Corolla Cross:
  * Added stop and go function
* DNGA/TNGA:
  * Complete retune for Myvi, Alza, Veloz & Ativa
  * Lateral retune for Lexus NX

Version 9.5-longOverhaul  (2023-12-08)
=========================
* Support BYD Atto 3
* Fix intermittent disengage for BYD, Hondas and Proton
* Vellphard:
  * Reduce jerks during high speed drives

Version 9.4-longOverhaul  (2023-10-06)
=========================
* Quiet mode sounds all safety related warnings
* Support for new KommuAssist1s
* Support Toyota Vios 2023
* TNGA/DNGA:
  * Reduce fighting torque during manual lane change
  * Revamp longitudinal control (quick braking reaction)
  * Fix steer limit warning
* TSS2:
  * Minor longitudinal retuning
* Proton:
  * Fix stock LKS settings except Auxiliary

Version 9.3-longOverhaul  (2023-07-10)
=========================
* Allow forced car profile selection through fix fingerprinting
* Increase reactiveness to lead accel for pure vision vehicles
* Retune the MPC
* TNGA/DNGA:
  * Fix lane departure warning disable icon
  * More reactive stock AEB
  * Increase stopping decel rate
* TSS2:
  * Fix steer fault error during manual input
  * Throttle max RPM for Alphard Vellfire above 4k RPM

Version 9.1-longOverhaul  (2023-02-25)
=========================
 * Fix firmware caused device error during startup
 * x50 (credits to Ryan & Darren):
   * Added software package 'ignore-ignition-line' for x50
   * Improved torque
   * Improved control stability at higher speeds
   * Reduced steering fighting torque
 * Perodua PSD:
   * Reduced steering fighting torque during manual override
   * Fix Alza/Veloz standstill brake and resume
   * Fix stock lane departure prevention and warning
   * Fix stock ADAS settings
   * Fix Alza/Veloz fingerprint clash with Ativa
   * Retuned lateral for better curve handling
   * Retuned longitudinal for Ativa and Alza
 * Perodua MG3 (credits to Kenneth):
   * More aggresive gas to close distance at higher speed

Version 9-longOverhaul  (2023-02-25)
=========================
 * Fix firmware caused device error during startup
 * x50 (credits to Ryan & Darren):
   * Added software package 'ignore-ignition-line' for x50
   * Improved torque
   * Improved control stability at higher speeds
   * Reduced steering fighting torque
 * Perodua PSD:
   * Reduced steering fighting torque during manual override
   * Fix Alza/Veloz standstill brake and resume
   * Fix stock lane departure prevention and warning
   * Fix stock ADAS settings
   * Fix Alza/Veloz fingerprint clash with Ativa
   * Retuned lateral for better curve handling
   * Retuned longitudinal for Ativa and Alza
 * Perodua MG3 (credits to Kenneth):
   * More aggresive gas to close distance at higher speed

Version 8-longOverhaul  (2022-11-30)
=========================
 * User configurable fan speed, power saver entry duration, vehicle skew and stopping distance
 * Perodua PSD: Fix brake bleeding
 * Perodua MG3: Fix distance and steering oscillation
 * Proton X50: Can disengage through cruise ready button
 * Proton X50: Hand touch warning if set to ICC instead of ACC
 * Added firmware recovery mechanism after a failed firmware flash
 * Support Perodua Alza (AV)
 * Support Toyota Veloz (AT)
 * Support Honda Civic 2022
 * Support Hyundai Ioniq HEV Plus 2017

Version 7-longOverhaul  (2022-10-25)
=========================
 * Recalculate MPC to allow harsher brakes for closer distance profiles
 * Added power saver toggle to shutdown device after 15 minutes of idle
 * Offset camera skew so that vehicles will slightly (negligible) lean to the right
 * Fix device re-registration problem after a reinstall
 * Toyota: Adjustable lead car follow profile
 * Perodua PSD: Stop-and-go function
 * Perodua PSD: Faster acceleration from standstill
 * Supported Proton X50 (Flagship)
 * Supported Honda City 2020 (V-Sensing)
 * Supported CRV 2020 (1.5 TCP, Black)
 * Supported Corolla Cross Hybrid

Version 6-longOverhaul  (2022-09-02)
=========================
 * New distance profile
 * Added more internal QC tools and vehicle porting tools

Version 5-longOverhaul  (2022-08-15)
=========================
 * bukapilot speed display matches stock speedometer
 * Cleaner settings UI
 * Rework of GPS time sync
 * New package feature thanks to @benmasato
 * Toyota: Enable stock Lane Departure Prevention when not engaged
 * Perodua Ativa & MFL: Adjustable lead car follow profile
 * From upstream openpilot v0.8.13:
   *  New driving model from upstream which is now trained on 1 million minutes of driving data
   *  Fixed lead training making lead predictions significantly more accurate
   *  Combined longitudinal planning now happens in a single MPC system
   *  New vision based forward collision warning
   *  New alert sounds
   *  New MPC acceleration lag compensation
   *  Fixed vehicleModelInvalid triggering due to false positives


Version 4-firstbatch  (2022-07-04)
=========================
 * Update UI: Training guide page, font change, add offroad alerts
 * Allow vision to leave power saver mode through ignition button and manual power on
 * Solve intermittent device power off
 * Change speed display to show GPS speed, NOT odometry speed
 * Toyota Alphard: Fix revving and wheelspeed scaling
 * Perodua PSD: Smoother low pump noise braking (lesser jerks)
 * Perodua PSD: Solve pump pressure bleeding issue
 * Perodua PSD: Enable stock Lane Departure Prevention when not engaged
 * Perodua Semi ACC: Add lateral tunes presets through settings
 * Support Toyota Corolla Cross 2021

Version 3-firstbatch  (2022-06-06)
=========================
 * Add full powersaver mode to 13 hours, battery can last at least 2 weeks idle with KommuAssist in power saver
 * Add stock ACC option in settings (only for Perodua)
 * Perodua PSD: Improve longitudinal
 * Perodua PSD: Add stock HUD warnings for front departure, forward collision warning & braking
 * Perodua PSD: Revert cruise speed set logic back to stock behaviour
 * Perodua PSD: Add brake pressure bleed warning
 * Perodua MG3: Fix odometer scaling
 * Increase standstill braking distance for all vehicles
 * QC: Add pre-fulfillment QC test
 * Fix minor UI display problems and release note popup
 * Add file corruption recovery mechanism during scons build

Version 2-firstbatch  (2022-05-11)
=========================
 * First bukapilot prebuilt release
 * Perodua: Remove horseriding
 * Perodua: 1.5x better longitudinal
 * Bug: Remove intermittent controls mismatch

Version 1-firstbatch  (2022-04-26)
=========================
 * Initial release of bukapilot based on openpilot v0.8.6
 * Supported Perodua Ativa 2020 - 2022 (AV)
 * Supported Perodua Myvi FL 2022 (AV)
 * Supported Toyota Alphard/Hybrid 2019-20
 * Supported Toyota Camry/Hybrid 2021-22
 * Supported Toyota Corolla Altis/Hybrid 2020-P
 * Supported Toyota Prius 2021-22
 * Supported Lexus ES/ES Hybrid 2019-21
 * Supported Lexus NX 2020
 * Supported Lexus UX/RX/Hybrid 2020-21
 * Supported Axia with KommuActuator2019 - 2022 (GXtra, Style, SE, AV)
 * Supported Bezza with KommuActuator 2020 - 2022 (X, AV)
 * Supported Myvi Gen 3 Pre-FL with KommuActuator 2017 - 2021 (G, X, H, AV)
