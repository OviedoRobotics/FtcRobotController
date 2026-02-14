#!/usr/bin/env python3
"""
🎯 LIMELIGHT QUICK OPTIMIZER - The Masterpiece
Optimizes AprilTag detection stability in ~2 minutes

Usage:
    python3 final_optimize.py           # Normal output
    python3 final_optimize.py -v        # Verbose output
    python3 final_optimize.py --verbose # Verbose output
"""

import requests
import time
import statistics
import sys

# ============================================================
# CONFIGURATION
# ============================================================
LIMELIGHT_IP = '172.29.0.1'
BASE = f'http://{LIMELIGHT_IP}:5807'

# Tags to look for (will auto-detect)
TARGET_TAGS = [20, 24]

# Safe baseline settings (user's proven settings)
SAFE_SETTINGS = {
    'exposure': 3200,
    'sensor_gain': 8.2,
    'fiducial_refine_method': 1,
    'black_level': 20,
    'sharpening': 0,
    'red_balance': 1280,
    'blue_balance': 1500,
}

# Verbose mode
VERBOSE = '-v' in sys.argv or '--verbose' in sys.argv

# ============================================================
# CORE FUNCTIONS
# ============================================================
def log(msg, force=False):
    """Print message (always if force, otherwise only in verbose mode)"""
    if force or VERBOSE:
        print(msg)

def apply_settings(settings):
    """Apply settings to camera"""
    try:
        requests.post(f'{BASE}/update-pipeline', json=settings, timeout=2)
        time.sleep(0.15)
        return True
    except:
        return False

def measure(samples=40, target_tag=None):
    """
    Measure stability for target tag(s)
    Returns: (z_std_mm, detection_rate, detected_tag_id)
    """
    z_vals = []
    detected_tag = None

    for _ in range(samples):
        try:
            r = requests.get(f'{BASE}/results', timeout=1)
            data = r.json()
            for fid in data.get('Fiducial', []):
                fid_id = fid.get('fID')
                # Accept specific tag or any target tag
                if target_tag and fid_id != target_tag:
                    continue
                if not target_tag and fid_id not in TARGET_TAGS:
                    continue

                t6t = fid.get('t6t_cs', [])
                if t6t and len(t6t) >= 3:
                    z_vals.append(t6t[2])
                    detected_tag = fid_id
                    break  # One detection per sample
        except:
            pass
        time.sleep(0.025)

    if len(z_vals) < 5:
        return 999, 0, detected_tag

    return statistics.stdev(z_vals) * 1000, len(z_vals) / samples * 100, detected_tag

def find_good_range(param_name, values, base_settings, tag_id):
    """
    Test values and find the "good range" (within 1.5x of best)
    Returns: (center_value, center_z_std, all_results)
    """
    results = []

    for val in values:
        test = base_settings.copy()
        test[param_name] = val
        apply_settings(test)
        z, det, _ = measure(30, tag_id)
        results.append((val, z, det))
        log(f'      {param_name}={val}: {z:.1f}mm ({det:.0f}%)')

    # Filter good detections
    good = [(v, z, d) for v, z, d in results if d > 70]
    if not good:
        good = [(v, z, d) for v, z, d in results if d > 40]
    if not good:
        return base_settings.get(param_name), 999, results

    # Find best and define threshold
    best_z = min(z for v, z, d in good)
    threshold = best_z * 1.5

    # Get range of values within threshold
    good_range = [v for v, z, d in good if z <= threshold]

    if good_range:
        center = good_range[len(good_range) // 2]
        center_z = next(z for v, z, d in results if v == center)
        return center, center_z, results
    else:
        best_val = min(good, key=lambda x: x[1])[0]
        return best_val, best_z, results

# ============================================================
# MAIN OPTIMIZATION
# ============================================================
def main():
    start_time = time.time()

    print('=' * 60)
    print('🎯 LIMELIGHT QUICK OPTIMIZER')
    print('=' * 60)

    # ----------------------------------------------------------
    # PHASE 1: Detect tag and establish baseline
    # ----------------------------------------------------------
    print('\n📡 PHASE 1: Detecting tag and baseline...')

    apply_settings(SAFE_SETTINGS)
    z, det, tag_id = measure(50)

    if det < 30:
        print(f'   ⚠️  Low detection ({det:.0f}%). Trying alternative settings...')
        # Try higher exposure
        alt = SAFE_SETTINGS.copy()
        alt['exposure'] = 2000
        apply_settings(alt)
        z, det, tag_id = measure(50)

    if det < 30 or tag_id is None:
        print('   ❌ Cannot detect tag! Check camera view.')
        return

    print(f'   ✅ Detected Tag {tag_id}')
    print(f'   📊 Baseline: {z:.2f}mm stability ({det:.0f}% detection)')

    best_settings = SAFE_SETTINGS.copy()
    best_z = z

    # ----------------------------------------------------------
    # PHASE 2: Coarse grid search for exposure + gain
    # ----------------------------------------------------------
    print('\n🔍 PHASE 2: Coarse parameter search...')

    # These ranges are based on what we learned works for this camera
    # Higher exposure + lower gain = cleaner image
    exposures = [2400, 2600, 2800, 3000, 3200]
    gains = [6, 7, 8, 9, 10]

    coarse_results = []
    total = len(exposures) * len(gains)
    count = 0

    for exp in exposures:
        for gain in gains:
            count += 1
            settings = best_settings.copy()
            settings['exposure'] = exp
            settings['sensor_gain'] = gain
            apply_settings(settings)
            z, det, _ = measure(25, tag_id)
            coarse_results.append((exp, gain, z, det))

            marker = '⭐' if det > 80 and z < best_z else ''
            log(f'   [{count:2d}/{total}] exp={exp} gain={gain}: {z:.1f}mm ({det:.0f}%) {marker}')

            if det > 80 and z < best_z:
                best_z = z
                best_settings['exposure'] = exp
                best_settings['sensor_gain'] = gain

    # Find good ranges
    good_exp = [e for e, g, z, d in coarse_results if d > 80 and z < best_z * 1.5]
    good_gain = [g for e, g, z, d in coarse_results if d > 80 and z < best_z * 1.5]

    if good_exp:
        best_settings['exposure'] = good_exp[len(good_exp) // 2]
    if good_gain:
        best_settings['sensor_gain'] = good_gain[len(good_gain) // 2]

    print(f'   ✅ Best so far: exp={best_settings["exposure"]} gain={best_settings["sensor_gain"]} ({best_z:.2f}mm)')

    # ----------------------------------------------------------
    # PHASE 3: Fine-tune secondary parameters
    # ----------------------------------------------------------
    print('\n🎛️  PHASE 3: Fine-tuning secondary parameters...')

    # Refine method
    log('   Testing refine methods...')
    center, z, _ = find_good_range(
        'fiducial_refine_method', [0, 1, 2, 3], best_settings, tag_id
    )
    best_settings['fiducial_refine_method'] = center
    if z < best_z:
        best_z = z
    refine_names = {0: 'None', 1: 'Subpixel', 2: 'Decode', 3: 'Pose'}
    print(f'   ✅ Refine method: {refine_names[center]}')

    # Black level
    log('   Testing black levels...')
    center, z, _ = find_good_range(
        'black_level', [15, 18, 20, 22, 25], best_settings, tag_id
    )
    best_settings['black_level'] = center
    if z < best_z:
        best_z = z
    print(f'   ✅ Black level: {center}')

    # Sharpening
    log('   Testing sharpening...')
    center, z, _ = find_good_range(
        'sharpening', [0, 0.05, 0.1, 0.15], best_settings, tag_id
    )
    best_settings['sharpening'] = center
    if z < best_z:
        best_z = z
    print(f'   ✅ Sharpening: {center}')

    # Red balance
    log('   Testing red balance...')
    center, z, _ = find_good_range(
        'red_balance', [1180, 1230, 1280, 1330, 1380], best_settings, tag_id
    )
    best_settings['red_balance'] = center
    if z < best_z:
        best_z = z
    print(f'   ✅ Red balance: {center}')

    # Blue balance
    log('   Testing blue balance...')
    center, z, _ = find_good_range(
        'blue_balance', [1400, 1450, 1500, 1550, 1600], best_settings, tag_id
    )
    best_settings['blue_balance'] = center
    if z < best_z:
        best_z = z
    print(f'   ✅ Blue balance: {center}')

    # ----------------------------------------------------------
    # PHASE 4: Quick cycling refinement
    # ----------------------------------------------------------
    print('\n🔄 PHASE 4: Cycling refinement...')

    # One more pass on exp/gain with tighter ranges
    exp_val = best_settings['exposure']
    gain_val = best_settings['sensor_gain']

    exp_range = [exp_val - 200, exp_val - 100, exp_val, exp_val + 100, exp_val + 200]
    exp_range = [e for e in exp_range if 800 <= e <= 3300]

    gain_range = [gain_val - 1, gain_val - 0.5, gain_val, gain_val + 0.5, gain_val + 1]
    gain_range = [g for g in gain_range if 4 <= g <= 30]

    log('   Fine-tuning exposure...')
    center, z, _ = find_good_range('exposure', exp_range, best_settings, tag_id)
    best_settings['exposure'] = center
    if z < best_z:
        best_z = z

    log('   Fine-tuning gain...')
    center, z, _ = find_good_range('sensor_gain', gain_range, best_settings, tag_id)
    best_settings['sensor_gain'] = center
    if z < best_z:
        best_z = z

    print(f'   ✅ Refined: exp={best_settings["exposure"]} gain={best_settings["sensor_gain"]}')

    # ----------------------------------------------------------
    # PHASE 5: Verification and apply
    # ----------------------------------------------------------
    print('\n✅ PHASE 5: Verification...')

    apply_settings(best_settings)
    time.sleep(0.3)

    verify_results = []
    for i in range(3):
        z, det, _ = measure(50, tag_id)
        verify_results.append((z, det))
        print(f'   Round {i+1}: {z:.2f}mm ({det:.0f}%)')

    final_z = statistics.mean([z for z, d in verify_results])
    final_det = statistics.mean([d for z, d in verify_results])

    elapsed = time.time() - start_time

    # ----------------------------------------------------------
    # FINAL REPORT
    # ----------------------------------------------------------
    print('\n' + '=' * 60)
    print('🏆 OPTIMIZATION COMPLETE')
    print('=' * 60)
    print(f'\n⏱️  Time: {elapsed:.1f} seconds')
    print(f'🏷️  Tag: {tag_id}')
    print(f'\n📊 Final Stability: {final_z:.2f}mm ({final_det:.0f}% detection)')
    print(f'\n⚙️  Optimal Settings:')
    print(f'   exposure: {best_settings["exposure"]}')
    print(f'   sensor_gain: {best_settings["sensor_gain"]}')
    print(f'   fiducial_refine_method: {best_settings["fiducial_refine_method"]} ({refine_names[best_settings["fiducial_refine_method"]]})')
    print(f'   black_level: {best_settings["black_level"]}')
    print(f'   sharpening: {best_settings["sharpening"]}')
    print(f'   red_balance: {best_settings["red_balance"]}')
    print(f'   blue_balance: {best_settings["blue_balance"]}')
    print(f'\n✅ Settings applied to camera!')
    print('=' * 60)

if __name__ == '__main__':
    main()