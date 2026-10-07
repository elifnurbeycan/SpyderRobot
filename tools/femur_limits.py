# Hexapod - Femur Motor Mekanik Limit Tablosu
# ST3020 Servo Motorlar | Arduino Mega 2560 | TX1(18)/RX1(19)
# Ölçüm tarihi: 2026-07-20

FEMUR_LIMITS = {
    #  ID : (min_pos, max_pos, merkez_pos)
     2: (3046, 4090, 3568),   # Femur 1 | Aralık: 1044 adım |  91.8°
     5: (1487, 3395, 2441),   # Femur 2 | Aralık: 1908 adım | 167.7°
     8: (3220, 4091, 3655),   # Femur 3 | Aralık:  871 adım |  76.6°
    11: (2384, 4091, 3237),   # Femur 4 | Aralık: 1707 adım | 150.0°
    14: (3362, 4091, 3726),   # Femur 5 | Aralık:  729 adım |  64.1°
    17: (2303, 4090, 3196),   # Femur 6 | Aralık: 1787 adım | 157.1°
}

# Yardimci: adim -> derece
# 1 adim = 360 / 4096 ≈ 0.0879°
STEPS_PER_DEG = 4096 / 360.0

def steps_to_deg(steps):
    return steps / STEPS_PER_DEG

def deg_to_steps(deg):
    return int(deg * STEPS_PER_DEG)

if __name__ == "__main__":
    print(f"{'Motor':<10} {'ID':>4} {'MIN':>6} {'MAX':>6} {'MERKEZ':>8} {'Aralık':>8} {'Derece':>8}")
    print("-" * 60)
    names = ["Femur 1", "Femur 2", "Femur 3", "Femur 4", "Femur 5", "Femur 6"]
    for (mid, (mn, mx, ctr)), name in zip(FEMUR_LIMITS.items(), names):
        aralik = mx - mn
        derece = steps_to_deg(aralik)
        print(f"{name:<10} {mid:>4} {mn:>6} {mx:>6} {ctr:>8} {aralik:>8} {derece:>7.1f}°")
