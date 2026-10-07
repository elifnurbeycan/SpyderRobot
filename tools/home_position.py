# Hexapod - Motor Pozisyon Kütüphanesi
# Kayıt tarihi: 2026-07-21
#
# ST3020 (femur)  : Serial1 (TX=18, RX=19) | 0-4095 | IDs: 2,5,8,11,14,17
# AX-12A (tibia/coxa): Serial2 (TX=16, RX=17) | 0-1023 | IDs: 1-18 (çift=coxa, tek=tibia)
#
# Motor rolleri (AX-12A):
#   COXA  (yatay dönüş) : 3, 6, 9, 10, 13, 16  → sıfır değişim beklenir (stance'te dokunulmadı)
#   TİBİA sol bacaklar  : 1, 4, 7   → aşağı hareket = encoder AZALIR (-)
#   TİBİA sağ bacaklar  : 12, 15, 18 → aşağı hareket = encoder ARTAR (+) [aynalı montaj]
#     12=ön sağ, 15=orta sağ, 18=arka sağ
#
# Not: ST3020 ID 8 ve 14 home pozisyonu 0'a yakın (159, 308).
#   Stance'te wrap geçti → raw fark büyük görünür ama gerçek hareket ~50°.

ST3020_HOME = {   # femur motorları
     2: 3918,    # Femur 1
     5: 2496,    # Femur 2
     8:  159,    # Femur 3
    11: 3634,    # Femur 4
    14:  308,    # Femur 5
    17: 3465,    # Femur 6
}

AX12_HOME = {     # coxa + tibia motorları
     1:  533,
     3:  564,
     4:  765,
     6:  526,
     7:  484,
     9:  518,
    10:  822,
    12:  541,
    13: 1023,
    15:  827,
    16:  493,
    18:  196,
}

# Örümcek duruşu — femur + tibia hafif aşağı
ST3020_STANCE = {   # femur motorları
     2: 3283,    # Femur 1
     5: 1918,    # Femur 2
     8: 3616,    # Femur 3
    11: 2997,    # Femur 4
    14: 3716,    # Femur 5  (2026-07-22: 14/17 motor takasi sonrasi yeniden olculdu)
    17: 2994,    # Femur 6
}

AX12_STANCE = {     # coxa + tibia motorları
     1:  465,
     3:  563,
     4:  699,
     6:  527,
     7:  403,
     9:  518,
    10:  821,
    12:  593,
    13: 1023,
    15:  908,
    16:  496,
    18:  303,
}

# Coxalar öne — tüm bacaklar robotun önü yönünde
# Femur + tibia stance ile aynı, sadece coxalar döndü
ST3020_COXA_FWD = ST3020_STANCE  # femur değişmedi

AX12_COXA_FWD = {   # coxa öne döndü, tibia stance ile aynı
     1:  465,   # tibia — değişmedi
     3:  979,   # coxa  ← öne
     4:  699,   # tibia — değişmedi
     6:  362,   # coxa  ← öne
     7:  403,   # tibia — değişmedi
     9:  325,   # coxa  ← öne
    10:  988,   # coxa  ← öne
    12:  593,   # tibia — değişmedi
    13:    0,   # coxa  ← öne
    15:  908,   # tibia — değişmedi
    16:  630,   # coxa  ← öne
    18:  303,   # tibia — değişmedi
}

# Tüm motorlar birleşik (tip bilgisiyle)
ALL_HOME = {
    'st3020': ST3020_HOME,
    'ax12':   AX12_HOME,
}

# Femur limit tablosu (referans için)
FEMUR_LIMITS = {
     2: (3046, 4090),   # Femur 1 | 91.8°
     5: (1487, 3395),   # Femur 2 | 167.7°
     8: (3220, 4091),   # Femur 3 | 76.6°
    11: (2384, 4091),   # Femur 4 | 150.0°
    14: (3362, 4091),   # Femur 5 | 64.1°
    17: (2303, 4090),   # Femur 6 | 157.1°
}

if __name__ == '__main__':
    print("=== ST3020 HOME ===")
    for mid, pos in ST3020_HOME.items():
        lim = FEMUR_LIMITS.get(mid, (0, 4095))
        pct = (pos - lim[0]) / (lim[1] - lim[0]) * 100 if lim[1] > lim[0] else 0
        print(f"  ID {mid:2d}: {pos:4d}   ({pct:5.1f}% aralıkta)")

    print("\n=== AX-12A HOME ===")
    for mid, pos in AX12_HOME.items():
        deg = pos * 300 / 1023
        print(f"  ID {mid:2d}: {pos:4d}   ({deg:5.1f}°)")
