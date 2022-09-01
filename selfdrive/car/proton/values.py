# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car

from collections import defaultdict
Ecu = car.CarParams.Ecu

# Todo
HUD_MULTIPLIER = 1.035

class CAR:
  X50 = "PROTON X50"

FINGERPRINTS = {
  CAR.X50: [{
    132: 8, 133: 8, 224: 8, 249: 8, 250: 8, 275: 8, 290: 8, 291: 8, 292: 8, 293: 8, 336: 8, 496: 8, 498: 8, 536: 8, 608: 8, 621: 8, 641: 8, 644: 8, 645: 8, 646: 8, 647: 8, 648: 8, 650: 8, 652: 8, 658: 8, 672: 8, 673: 8, 674: 8, 675: 8, 676: 8, 677: 8, 679: 8, 680: 8, 681: 8, 682: 8, 683: 8, 684: 8, 685: 8, 686: 8, 687: 8, 691: 8, 692: 8, 753: 8, 754: 8, 763: 8, 764: 8, 765: 8, 880: 8, 896: 8, 912: 8, 993: 8, 994: 8, 1005: 8, 1006: 8, 1008: 8, 1009: 8, 1014: 8, 1015: 8, 1026: 8, 1031: 8, 1038: 8, 1040: 8, 1045: 8, 2015: 8, 2024: 8
  }, {
    132: 8, 133: 8, 224: 8, 249: 8, 250: 8, 275: 8, 290: 8, 291: 8, 292: 8, 293: 8, 336: 8, 401: 8, 418: 8, 419: 8, 422: 8, 423: 8, 432: 8, 434: 8, 496: 8, 498: 8, 536: 8, 608: 8, 621: 8, 641: 8, 644: 8, 645: 8, 646: 8, 647: 8, 648: 8, 650: 8, 658: 8, 672: 8, 673: 8, 674: 8, 675: 8, 676: 8, 677: 8, 679: 8, 680: 8, 681: 8, 682: 8, 683: 8, 684: 8, 685: 8, 686: 8, 687: 8, 691: 8, 692: 8, 736: 8, 753: 8, 754: 8, 763: 8, 764: 8, 765: 8, 880: 8, 896: 8, 912: 8, 993: 8, 994: 8, 1005: 8, 1006: 8, 1008: 8, 1009: 8, 1014: 8, 1015: 8, 1026: 8, 1031: 8, 1038: 8, 1040: 8, 1045: 8, 1945: 8, 1961: 8, 2015: 8, 2024: 8, 2025: 8
  }, {
    32: 8, 33: 8, 34: 8, 35: 8, 36: 8, 37: 8, 38: 8, 39: 8, 40: 8, 41: 8, 42: 8, 43: 8, 44: 8, 45: 8, 46: 8, 47: 8, 48: 8, 49: 8, 50: 8, 51: 8, 52: 8, 53: 8, 54: 8, 55: 8, 56: 8, 57: 8, 58: 8, 59: 8, 60: 8, 61: 8, 62: 8, 63: 8, 64: 8, 65: 8, 66: 8, 67: 8, 68: 8, 69: 8, 70: 8, 71: 8, 80: 8, 81: 8, 82: 8, 83: 8, 84: 8, 85: 8, 86: 8, 87: 8, 88: 8, 89: 8, 90: 8, 91: 8, 92: 8, 93: 8, 94: 8, 95: 8, 96: 8, 97: 8, 98: 8, 99: 8, 100: 8, 101: 8, 102: 8, 103: 8, 104: 8, 105: 8, 106: 8, 107: 8, 108: 8, 109: 8, 110: 8, 111: 8, 112: 8, 113: 8, 114: 8, 115: 8, 116: 8, 117: 8, 118: 8, 119: 8, 128: 8, 130: 8, 132: 8, 133: 8, 151: 8, 224: 8, 275: 8, 288: 8, 289: 8, 290: 8, 291: 8, 292: 8, 293: 8, 294: 8, 295: 8, 298: 8, 304: 8, 305: 8, 336: 8, 400: 8, 401: 8, 417: 8, 418: 8, 419: 8, 422: 8, 423: 8, 432: 8, 434: 8, 482: 8, 496: 8, 497: 8, 536: 8, 608: 8, 621: 8, 641: 8, 643: 8, 644: 8, 645: 8, 646: 8, 650: 8, 654: 8, 658: 8, 672: 8, 676: 8, 679: 8, 680: 8, 682: 8, 686: 8, 692: 8, 736: 8, 753: 8, 784: 8, 896: 8, 912: 8, 993: 8, 994: 8, 1008: 8, 1009: 8, 1033: 8, 1036: 8, 1039: 8, 1058: 8
  }],
}

DBC = {
  CAR.X50: dbc_dict('proton_general_pt', None),
}

# Todo
BRAKE_SCALE = defaultdict(lambda: 1, {CAR.X50: 1})
# Todo
GAS_SCALE = defaultdict(lambda: 1, {CAR.X50: 1})

