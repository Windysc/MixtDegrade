import xml.etree.ElementTree as ET
import math
import os
import glob


# 1. CONFIGURATION & MATERIAL PARAMETERS


# Folder Path containing lane XML files
DATA_FOLDER = '.'

# File pattern for lane files (e.g., e1i_0.xml, e1i_1.xml, etc.)
FILE_PATTERN = 'e1i_*.xml'

# Pavement Temperature (Assumed constant for this script)
TEMP_C = 20.0  # degrees Celsius


# VEHICLE AXLE CONFIGURATION
# 1. Load distribution across axles
# 2. Axle interaction effects (tandem/tridem)
# 3. ESAL (Equivalent Single Axle Load) conversion

VEHICLE_CONFIGS = {
    "passenger": {
        "total_weight": 15,      
        "axles": [(7, 'single'), (8, 'single')],  # Front, Rear
        "description": "Passenger car"
    },
    "DEFAULT_VEHTYPE": {
        "total_weight": 15,     
        "axles": [(7, 'single'), (8, 'single')],
        "description": "Default passenger vehicle"
    },
    "Autonomous": {
        "total_weight": 18,      
        "axles": [(8, 'single'), (10, 'single')],
        "description": "Autonomous vehicle (passenger car)"
    },
    "Truck": {
        "total_weight": 180,     
        "axles": [(60, 'single'), (120, 'tandem')],  
        "description": "Medium truck (2-axle with tandem rear)"
    },
    "HGV": {
        "total_weight": 400,     
        "axles": [(70, 'single'), (180, 'tandem'), (150, 'tandem')],
        "description": "Heavy goods vehicle (tractor + semi-trailer)"
    },
    "Bus": {
        "total_weight": 180,    
        "axles": [(70, 'single'), (110, 'single')], 
        "description": "City bus (2-axle)"
    },
    "Trailer": {
        "total_weight": 440,    
        "axles": [(70, 'single'), (180, 'tandem'), (190, 'tridem')],
        "description": "Heavy articulated truck"
    }
}

# ESAL factors for multi-axle groups
# Based on AASHTO load equivalency factors
AXLE_TYPE_FACTORS = {
    'single': 1.0,      
    'tandem': 0.85,     
    'tridem': 0.75,     
}

STANDARD_AXLE_LOAD = 80.0  # 80 kN = 18,000 lbs (US standard)

# LOAD-BASED DAMAGE THRESHOLD

MINIMUM_DAMAGING_AXLE_LOAD = 1.0  # kN - axles below this cause negligible damage

# For loads between threshold and standard, apply 4th power scaling
# This captures the empirical AASHTO finding that light loads are nearly harmless

# MLET PAVEMENT STRUCTURE PARAMETERS
# Typical flexible pavement structure for medium traffic road

# Layer thicknesses (mm) - Realistic urban road
H_ASPHALT = 120.0     
H_BASE = 145.0       
H_SUBBASE = 220.0    

# Layer moduli at reference temperature 20°C (MPa)
E_ASPHALT_REF = 3500.0   
E_BASE = 300.0          
E_SUBBASE = 150.0        
E_SUBGRADE = 50.0        


NU_ASPHALT = 0.35
NU_BASE = 0.35
NU_SUBBASE = 0.35
NU_SUBGRADE = 0.45

# Tire parameters
TIRE_PRESSURE = 700.0    
TIRE_RADIUS = 107.0      

# SIGMOIDAL MASTER CURVE PARAMETERS
# log|E*| = delta + alpha / (1 + exp(beta + gamma * log(fr)))


SIG_DELTA = 1.30     # Lower asymptote (log MPa) 
SIG_ALPHA = 2.30     # Span 
SIG_BETA = -1.20     # horizontal position
SIG_GAMMA = -0.45    # slope

# WLF Temperature Shift Parameters (typical for asphalt)
WLF_C1 = 19.0        
WLF_C2 = 92.0        
T_REF = 20.0         

# FATIGUE MODEL PARAMETERS (Modified with Rest Period)
# N_f,i = a * (1/epsilon)^b * (1/E)^c * exp(d*T + f*RP) * SF
# Where:
#   - epsilon: tensile strain in MICROSTRAIN (µε)
#   - E: asphalt modulus (MPa)
#   - T: temperature (°C)
#   - RP: rest period between loads (SECONDS)
#   - SF: shift factor

FATIGUE_PARAMS = {
    'single': {
        'a': 8.593e16,   # Calibration constant
        'b': 2.769,      # Strain exponent
        'c': 1.230,      # Stiffness exponent
        'd': 0.046,      # Temperature coefficient (°C)
        'f': 1.062,      # Rest period coefficient (seconds)
        'R2': 0.723      # Model fit quality
    },
    'tandem': {
        'a': 2.823e14,
        'b': 2.353,
        'c': 0.974,
        'd': 0.043,
        'f': 1.360,
        'R2': 0.823
    },
    'tridem': {
        'a': 8.891e13,
        'b': 2.051,
        'c': 1.115,
        'd': 0.042,
        'f': 1.310,
        'R2': 0.866
    }
}

# Default to single-axle parameters for general calculations
FATIGUE_A = FATIGUE_PARAMS['single']['a']
FATIGUE_B = FATIGUE_PARAMS['single']['b']
FATIGUE_C = FATIGUE_PARAMS['single']['c']
FATIGUE_D = FATIGUE_PARAMS['single']['d']
FATIGUE_F = FATIGUE_PARAMS['single']['f']
SHIFT_FACTOR = 1.0      # Lab to field shift factor (calibration already includes field conditions)


ROAD_DIRECTIONS = {
    'Direction_A': ['e1i_0.xml', 'e1i_1.xml'],  
    'Direction_B': ['e1i_2.xml', 'e1i_3.xml'],  
}

# Lane distribution factors WITHIN each direction

LANE_FACTORS = {
    'e1i_0.xml': 0.60,  
    'e1i_1.xml': 0.40,   
    'e1i_2.xml': 0.40,   
    'e1i_3.xml': 0.60,   
}
DEFAULT_LANE_FACTOR = 0.50

# 2. AXLE LOAD AND ESAL FUNCTIONS


def get_vehicle_axles(v_type):
    """
    Get axle configuration for a vehicle type.
    Returns list of (axle_load_kN, axle_type) tuples.
    """
    if v_type in VEHICLE_CONFIGS:
        return VEHICLE_CONFIGS[v_type]["axles"]
    else:
        return [(8, 'single'), (10, 'tandem')]

def calculate_axle_esal(axle_load_kn, axle_type):
    """
    Calculate ESAL (Equivalent Single Axle Load) for one axle group.
    
    Uses the 4th power law: ESAL = (P/P_std)^4 * type_factor
    Where P_std = 80 kN (standard axle load)
    """
    if axle_load_kn <= 0:
        return 0.0

    load_ratio = axle_load_kn / STANDARD_AXLE_LOAD
    
    esal_base = load_ratio ** 4.0
    
    type_factor = AXLE_TYPE_FACTORS.get(axle_type, 1.0)
    
    return esal_base * type_factor

def calculate_vehicle_total_esal(v_type):
    """
    Calculate total ESAL contribution from all axles of a vehicle.
    
    Each axle pass contributes to pavement damage independently,
    but the damage from each depends on load and axle configuration.
    """
    axles = get_vehicle_axles(v_type)
    total_esal = 0.0
    
    for axle_load, axle_type in axles:
        total_esal += calculate_axle_esal(axle_load, axle_type)
    
    return total_esal

def get_equivalent_single_axle_load(v_type):
    """
    Convert vehicle's total ESAL to an equivalent single axle load.
    This is used for strain calculation - we calculate strain for
    an equivalent single axle that produces the same total damage.
    
    Returns: equivalent single axle load in kN
    """
    total_esal = calculate_vehicle_total_esal(v_type)
    
    if total_esal <= 0:
        return 0.0
    
    equiv_load = STANDARD_AXLE_LOAD * (total_esal ** 0.25)
    return equiv_load


# 3. MLET CALCULATION FUNCTIONS


def calculate_wlf_shift(temp_c):
    """
    Calculate WLF temperature shift factor.
    log(aT) = -C1 * (T - Tref) / (C2 + T - Tref)
    """
    delta_t = temp_c - T_REF
    if abs(WLF_C2 + delta_t) < 0.001:
        return 1.0
    log_at = -WLF_C1 * delta_t / (WLF_C2 + delta_t)
    return 10 ** log_at

def calculate_loading_frequency(speed_ms):
    """
    Calculate loading frequency based on vehicle speed.
    f = v / (2 * a), where a is contact radius
    """
    if speed_ms <= 0:
        return 1.0
    # Convert tire radius to meters
    a_m = TIRE_RADIUS / 1000.0
    freq = speed_ms / (2.0 * a_m)
    return freq

def calculate_asphalt_modulus(speed_ms, temp_c):
    """
    Calculate dynamic modulus using Sigmoidal Master Curve with WLF shift.
    """
    # Calculate loading frequency
    freq = calculate_loading_frequency(speed_ms)
    
    # Calculate temperature shift factor
    a_t = calculate_wlf_shift(temp_c)
    
    # Reduced frequency
    freq_r = freq * a_t
    
    # Sigmoidal master curve
    if freq_r <= 0:
        freq_r = 0.001
    
    log_fr = math.log10(freq_r)
    log_E = SIG_DELTA + SIG_ALPHA / (1.0 + math.exp(SIG_BETA + SIG_GAMMA * log_fr))
    
    E_star = 10 ** log_E
    
    # Bound to realistic values
    E_star = max(100.0, min(E_star, 20000.0))
    
    return E_star


# Axle spacing within multi-axle groups (in meters)
AXLE_SPACING = {
    'single': 0.0,       
    'tandem': 1.22,      
    'tridem': 3.22,      
}

INTER_AXLE_GROUP_SPACING = 4.5  

def mlet_strain_calculation(axle_load_kn, E_asphalt, speed_ms, axle_type='single'):
    """
    Multi-Layer Elastic Theory (MLET) based strain calculation with axle configuration.
    
    Uses the standard MLET formula with strain factor from published charts.
    Reference: Huang, Y.H. "Pavement Analysis and Design", 2nd Ed.
    
    For a circular load on a two-layer system, the horizontal tensile strain
    at the bottom of the surface layer is given by:
    
    epsilon_t = (1 + nu) * (q/E1) * F_epsilon
    
    where F_epsilon is a strain factor from MLET charts depending on:
    - a/h1 (contact radius to AC thickness ratio)
    - E1/E2 (modular ratio)
    
    For multi-axle groups (tandem, tridem), the load is distributed across
    multiple axle lines, which modifies the strain response:
    - Tandem: 2 axles share the total load, spaced ~1.22m apart
    - Tridem: 3 axles share the total load, spaced ~1.22m apart each
    
    The strain calculation uses load per axle with superposition effects
    to account for overlapping stress fields from adjacent axles.
    
    Speed is used to calculate the inter-axle rest period within multi-axle groups:
    - rest_between_axles = spacing / speed
    - This affects the healing/recovery between consecutive axle loadings
    
    Args:
        axle_load_kn: Total axle group load in kN
        E_asphalt: Asphalt modulus in MPa
        speed_ms: Vehicle speed in m/s (used for inter-axle rest period calculation)
        axle_type: 'single', 'tandem', or 'tridem'
    
    Returns:
        tuple: (strain in absolute units, inter_axle_rest_period in seconds)
               For single axles, inter_axle_rest_period is 0.0
    """
    # Determine number of axles in the group and load per axle
    if axle_type == 'tandem':
        num_axles = 2
        load_per_axle = axle_load_kn / num_axles
    elif axle_type == 'tridem':
        num_axles = 3
        load_per_axle = axle_load_kn / num_axles
    else:  # single
        num_axles = 1
        load_per_axle = axle_load_kn
    
    # Get axle spacing for this type
    spacing = AXLE_SPACING.get(axle_type, 0.0)
    
    # Calculate inter-axle rest period based on speed
    # This is the time between consecutive axles passing over the same point
    # rest_period = distance / speed
    if speed_ms > 0 and spacing > 0:
        inter_axle_rest = spacing / speed_ms  # seconds
    else:
        inter_axle_rest = 0.0
    
    # Calculate contact radius from load per axle and pressure
    P = load_per_axle * 1000.0  # Load per axle in N
    q = TIRE_PRESSURE * 1000.0  # Contact pressure in Pa
    
    # Contact radius from load/pressure relationship
    a = math.sqrt(P / (math.pi * q))  # m
    
    # Layer thickness
    h1 = H_ASPHALT / 1000.0  # m
    
    # Moduli
    E1 = E_asphalt * 1e6  # Pa
    E2 = E_SUBGRADE * 1e6  # Pa
    
    # Dimensionless parameters
    a_h1 = a / h1  # typically 0.5 - 2.0
    E_ratio = E1 / E2  # typically 10 - 500
    
    # Strain factor from MLET (based on tabulated values)
    # This is a regression of KENLAYER/BISAR results
    # F_epsilon depends on a/h1 and E1/E2
    #
    # Based on MLET analysis (Huang, 2004) and calibration:
    # For a two-layer system, the tensile strain factor follows:
    # F_epsilon ≈ K * (a/h1)^0.8 / (E1/E2)^0.4
    # 
    # Where:
    # - a/h1: ratio of contact radius to AC layer thickness
    # - E1/E2: modular ratio (AC to subgrade)
    # - K ≈ 1.05 (calibration constant for 175 µε at 80kN standard axle)
    #
    # This gives realistic strain values:
    # - Light vehicles (8-10 kN axle): 70-80 microstrain
    # - Medium trucks (60-80 kN axle): 150-180 microstrain  
    # - Heavy trucks (120+ kN axle): 200-250 microstrain
    
    STRAIN_FACTOR_K = 1.05  # Calibration constant
    
    # Calculate strain factor
    F_epsilon = STRAIN_FACTOR_K * (a_h1 ** 0.8) / (E_ratio ** 0.4)
    
    # Bound to physically reasonable range (0.05 - 0.6)
    F_epsilon = max(0.05, min(F_epsilon, 0.6))
    
    # Single axle tensile strain at bottom of AC
    # epsilon_t = (1 + nu) * q / E1 * F_epsilon
    epsilon_single = (1.0 + NU_ASPHALT) * (q / E1) * F_epsilon
    
    # Multi-axle superposition factor
    # For tandem/tridem, calculate combined strain from multiple offset loads
    # The superposition accounts for overlapping stress fields
    
    if num_axles == 1:
        epsilon_t = epsilon_single
    else:
        # Multi-axle superposition calculation
        # Influence factor decreases with distance from load
        # Based on Boussinesq-type decay: I(r) ~ 1 / (1 + (r/h1)^2)^1.5
        
        superposition_sum = 1.0  # Contribution from first axle (directly above)
        
        for i in range(1, num_axles):
            r = i * spacing  # Distance to i-th axle
            r_h1 = r / h1
            # Influence decay factor (simplified from full MLET analysis)
            # This approximates strain contribution from offset loads
            influence = 1.0 / ((1.0 + (r_h1 ** 2)) ** 1.5)
            superposition_sum += influence
        
        # Combined strain: single axle strain × superposition factor
        # Note: This gives peak strain (worst case location)
        epsilon_t = epsilon_single * superposition_sum
    
    return epsilon_t, inter_axle_rest


def calculate_fatigue_life(strain, E_asphalt, temp_c, rest_period_seconds, axle_type='single'):
    """
    Calculate fatigue life using modified model with rest period.
    
    N_f,i = a * (1/epsilon)^b * (1/E)^c * exp(d*T + f*RP) * SF
    
    Where:
        - epsilon: tensile strain in MICROSTRAIN (µε) - research calibration units
        - E: asphalt modulus (MPa)
        - T: temperature (°C)
        - RP: rest period between loads (SECONDS)
        - SF: shift factor
    
    Parameters are axle-type specific based on Table 4-1 calibration:
        - Single-axle: a=8.593e16, b=2.769, c=1.230, d=0.046, f=1.062 (R²=0.723)
        - Tandem-axle: a=2.823e14, b=2.353, c=0.974, d=0.043, f=1.360 (R²=0.823)
        - Tridem-axle: a=8.891e13, b=2.051, c=1.115, d=0.042, f=1.310 (R²=0.866)
    
    Args:
        strain: Tensile strain (absolute units, e.g., 100e-6) - will be converted to microstrain
        E_asphalt: Asphalt modulus (MPa)
        temp_c: Temperature (°C)
        rest_period_seconds: Rest period since last load (seconds)
        axle_type: 'single', 'tandem', or 'tridem'
    
    Returns:
        N_f: Number of load repetitions to failure
    """
    if strain <= 0:
        return float('inf')
    
    # Get axle-type specific parameters
    params = FATIGUE_PARAMS.get(axle_type, FATIGUE_PARAMS['single'])
    a = params['a']
    b = params['b']
    c = params['c']
    d = params['d']
    f = params['f']
    
    # Convert strain from absolute to microstrain (research calibration uses µε)
    strain_microstrain = strain * 1e6
    
    # Rest period is already in seconds (research calibration uses seconds)
    rest_period = rest_period_seconds
    
    # N_f = a * (1/epsilon)^b * (1/E)^c * exp(d*T + f*RP) * SF
    # Which is equivalent to: a * epsilon^(-b) * E^(-c) * exp(d*T + f*RP) * SF
    
    strain_term = strain_microstrain ** (-b)   # (1/epsilon_µε)^b
    modulus_term = E_asphalt ** (-c)           # (1/E)^c
    temp_rest_term = math.exp(d * temp_c + f * rest_period)  # exp(d*T + f*RP)
    
    N_f = a * strain_term * modulus_term * temp_rest_term * SHIFT_FACTOR
    
    return max(N_f, 1.0)


# 3. LANE PROCESSING FUNCTION (MLET-based)

def process_lane_file(xml_file, verbose=False):
    """
    Process a single lane XML file using MLET-based calculations.
    
    Args:
        xml_file: Path to the XML file
        verbose: If True, print per-vehicle details
    
    Returns:
        Dictionary with lane statistics including strain statistics
    """

    total_damage = 0.0
    total_damage_no_healing = 0.0
    total_vehicles = 0
    previous_time = None
    
    # Track strain and stiffness statistics
    strain_values = []
    stiffness_values = []
    
    # Track vehicle types
    vehicle_type_counts = {}
    vehicle_type_damage = {}
    
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
    except Exception as e:
        print(f"Error reading {xml_file}: {e}")
        return None
    
    lane_name = os.path.basename(xml_file)
    
    if verbose:
        print(f"\n{'='*120}")
        print(f"Processing: {lane_name}")
        print(f"{'='*120}")
        print(f"{'VehID':<12} | {'Type':<15} | {'ESAL':<8} | {'Eq.Load':<10} | {'E_AC':<10} | {'Strain(µε)':<12} | {'N_f':<12} | {'Damage':<12}")
        print("-" * 130)
    
    for event in root.findall('instantOut'):
        if event.get('state') == 'enter':
            v_id = event.get('vehID')
            v_type = event.get('type')
            speed = float(event.get('speed'))
            current_time = float(event.get('time'))
            
            # Calculate total ESAL for this vehicle (all axles)
            total_esal = calculate_vehicle_total_esal(v_type)
            
            # Include ALL vehicles in damage calculation, including passenger cars
            # Even small vehicles contribute to cumulative fatigue damage
            # (Removed ESAL threshold filter to account for all traffic)
            
            # Get equivalent single axle load for strain calculation
            equiv_load = get_equivalent_single_axle_load(v_type)
            
            # Calculate rest period (time since last vehicle)
            if previous_time is not None:
                rest_period = current_time - previous_time
            else:
                rest_period = 0.0
            
            previous_time = current_time
            
            # MLET-based calculations using equivalent load
            E_asphalt = calculate_asphalt_modulus(speed, TEMP_C)
            
            # Calculate damage for EACH AXLE using axle-type specific parameters
            # This properly accounts for different fatigue behavior of single/tandem/tridem
            axles = get_vehicle_axles(v_type)
            vehicle_damage = 0.0
            vehicle_damage_no_rest = 0.0
            vehicle_N_f_avg = 0.0
            damaging_axles = 0
            
            # Calculate inter-axle-group rest period based on vehicle speed
            # This is the time between consecutive axle groups passing over a point
            # For a vehicle at speed v with axle groups spaced s apart: rest = s / v
            if speed > 0:
                inter_group_rest = INTER_AXLE_GROUP_SPACING / speed  # seconds
            else:
                inter_group_rest = 0.0
            
            for axle_idx, (axle_load, axle_type) in enumerate(axles):
                # Apply load threshold - axles below minimum cause negligible damage
                # This follows AASHTO finding that light vehicles cause statistically 
                if axle_load < MINIMUM_DAMAGING_AXLE_LOAD:
                    continue
                
                # Calculate strain for this specific axle load and axle type
                # Also returns inter-axle rest period (within tandem/tridem groups) based on speed
                axle_strain, intra_group_rest = mlet_strain_calculation(axle_load, E_asphalt, speed, axle_type)
                
                # Determine effective rest period for fatigue calculation:
                # - For tandem/tridem axles: use intra-group rest (time between axles within the group)
                # - For subsequent axle groups: use inter-group rest (time since previous axle group)
                # 
                # Note: We do NOT use inter-vehicle rest period here because:
                # 1. The fatigue model was calibrated for rest periods between load cycles
                # 2. Inter-vehicle rest can be many seconds, causing unrealistic healing
                # 3. The relevant rest period is between consecutive axle passes on same point
                
                if axle_type in ['tandem', 'tridem'] and intra_group_rest > 0:
                    # For multi-axle groups: use intra-group rest (within tandem/tridem)
                    effective_rest = intra_group_rest
                elif axle_idx > 0:
                    # For subsequent axle groups: use inter-group rest
                    effective_rest = inter_group_rest
                else:
                    # For first axle: no rest (or could use inter-vehicle rest if desired)
                    effective_rest = 0.0
                
                # Calculate N_f with axle-type specific parameters and appropriate rest period
                axle_N_f = calculate_fatigue_life(axle_strain, E_asphalt, TEMP_C, effective_rest, axle_type)
                axle_N_f_no_rest = calculate_fatigue_life(axle_strain, E_asphalt, TEMP_C, 0.0, axle_type)
                
                # Damage from this axle pass (Miner's rule: D = n/N_f where n=1)
                # Note: Load effect is already captured in strain -> N_f, so we use n=1 per axle pass
                # NOT multiplied by ESAL (that would double-count the load effect)
                if axle_N_f > 0:
                    vehicle_damage += 1.0 / axle_N_f
                    vehicle_damage_no_rest += 1.0 / axle_N_f_no_rest
                    vehicle_N_f_avg += axle_N_f
                    damaging_axles += 1
            
            # Average N_f for display purposes (only from damaging axles)
            N_f = vehicle_N_f_avg / damaging_axles if damaging_axles > 0 else float('inf')
            damage = vehicle_damage
            damage_no_rest = vehicle_damage_no_rest
            
            # For statistics, use equivalent load strain (single axle equivalent)
            strain, _ = mlet_strain_calculation(equiv_load, E_asphalt, speed, 'single')
            
            total_damage += damage
            total_damage_no_healing += damage_no_rest
            total_vehicles += 1
            
            # Track vehicle type counts and damage
            vehicle_type_counts[v_type] = vehicle_type_counts.get(v_type, 0) + 1
            vehicle_type_damage[v_type] = vehicle_type_damage.get(v_type, 0.0) + damage
            
            # Track statistics
            strain_values.append(strain * 1e6)  # Convert to microstrain
            stiffness_values.append(E_asphalt)
            
            if verbose:
                print(f"{v_id:<12} | {v_type:<15} | {total_esal:<8.3f} | {equiv_load:<10.1f} | {E_asphalt:<10.1f} | {strain*1e6:<12.2f} | {N_f:<12.2e} | {damage:<12.2e}")
    
    if verbose:
        print("-" * 130)
    
    # Calculate statistics
    avg_strain = sum(strain_values) / len(strain_values) if strain_values else 0
    avg_stiffness = sum(stiffness_values) / len(stiffness_values) if stiffness_values else 0
    
    return {
        'lane_name': lane_name,
        'total_vehicles': total_vehicles,
        'total_damage': total_damage,
        'total_damage_no_healing': total_damage_no_healing,
        'avg_strain_microstrain': avg_strain,
        'avg_stiffness_mpa': avg_stiffness,
        'vehicle_type_counts': vehicle_type_counts,
        'vehicle_type_damage': vehicle_type_damage
    }

# 4. MAIN EXECUTION (Multiple Lane Files)


xml_files = sorted(glob.glob(os.path.join(DATA_FOLDER, FILE_PATTERN)))

if not xml_files:
    print(f"No files matching '{FILE_PATTERN}' found in {DATA_FOLDER}")
    exit(1)

print("\n" + "="*130)
print("MLET-BASED PAVEMENT FATIGUE ANALYSIS - WITH MULTI-AXLE ESAL")
print("="*130)

print(f"\nPavement Structure:")
print(f"  Asphalt Layer:  {H_ASPHALT:.0f} mm, E_ref = {E_ASPHALT_REF:.0f} MPa")
print(f"  Base Layer:     {H_BASE:.0f} mm, E = {E_BASE:.0f} MPa")
print(f"  Subbase Layer:  {H_SUBBASE:.0f} mm, E = {E_SUBBASE:.0f} MPa")
print(f"  Subgrade:       E = {E_SUBGRADE:.0f} MPa")
print(f"\nAnalysis Temperature: {TEMP_C}°C")
print(f"Minimum Damaging Axle Load: {MINIMUM_DAMAGING_AXLE_LOAD} kN (lighter axles cause negligible damage)")

print(f"\nRoad Configuration (Bidirectional):")
for direction, lanes in ROAD_DIRECTIONS.items():
    print(f"  {direction}: {', '.join(lanes)}")

print(f"\nVehicle Axle Configuration & ESAL:")
print(f"  {'Type':<18} | {'Axles':<40} | {'Total ESAL':<10}")
print(f"  {'-'*18} | {'-'*40} | {'-'*10}")
for vtype, config in VEHICLE_CONFIGS.items():
    axle_str = ", ".join([f"{load}kN({atype[0]})" for load, atype in config['axles']])
    total_esal = calculate_vehicle_total_esal(vtype)
    print(f"  {vtype:<18} | {axle_str:<40} | {total_esal:<10.3f}")

# Process each lane file
lane_results = {}
vehicle_type_counts = {}  # Track counts by vehicle type
vehicle_type_damage = {}  # Track damage by vehicle type

for xml_file in xml_files:
    result = process_lane_file(xml_file, verbose=False)
    if result:
        lane_results[result['lane_name']] = result
        # Merge vehicle type counts and damage
        if 'vehicle_type_counts' in result:
            for vtype, count in result['vehicle_type_counts'].items():
                vehicle_type_counts[vtype] = vehicle_type_counts.get(vtype, 0) + count
        if 'vehicle_type_damage' in result:
            for vtype, dmg in result['vehicle_type_damage'].items():
                vehicle_type_damage[vtype] = vehicle_type_damage.get(vtype, 0.0) + dmg

# Print vehicle type breakdown
print("\n" + "="*130)
print("VEHICLE TYPE CONTRIBUTION TO DAMAGE (All Lanes Combined)")
print("="*130)
print(f"{'Vehicle Type':<18} | {'Count':<10} | {'Total Damage':<14} | {'% of Total':<12} | {'Damage/Vehicle':<14}")
print("-" * 82)

total_all_damage = sum(vehicle_type_damage.values())
for vtype in sorted(vehicle_type_damage.keys(), key=lambda x: vehicle_type_damage.get(x, 0), reverse=True):
    count = vehicle_type_counts.get(vtype, 0)
    dmg = vehicle_type_damage.get(vtype, 0)
    pct = (dmg / total_all_damage * 100) if total_all_damage > 0 else 0
    per_veh = dmg / count if count > 0 else 0
    print(f"{vtype:<18} | {count:<10} | {dmg:<14.5e} | {pct:<12.2f} | {per_veh:<14.5e}")


# 5. GENERATE SUMMARY REPORTS BY DIRECTION


print("\n" + "="*130)
print("LANE-BY-LANE RESULTS (MLET with Multi-Axle ESAL)")
print("="*130)
print(f"{'Lane':<15} | {'Vehicles':<10} | {'Avg ε (µε)':<12} | {'Avg E (MPa)':<12} | {'Lane Damage':<14} | {'Lane Factor':<12} | {'Weighted D':<14}")
print("-" * 130)

# Process each direction separately
direction_results = {}

for direction, lane_files in ROAD_DIRECTIONS.items():
    dir_weighted_damage = 0.0
    dir_weighted_damage_no_healing = 0.0
    dir_vehicles = 0
    
    for lane_file in lane_files:
        if lane_file in lane_results:
            r = lane_results[lane_file]
            lane_factor = LANE_FACTORS.get(r['lane_name'], DEFAULT_LANE_FACTOR)
            weighted_damage = r['total_damage'] * lane_factor
            
            dir_weighted_damage += weighted_damage
            dir_weighted_damage_no_healing += r['total_damage_no_healing'] * lane_factor
            dir_vehicles += r['total_vehicles']
            
            print(f"{r['lane_name']:<15} | {r['total_vehicles']:<10} | {r['avg_strain_microstrain']:<12.2f} | {r['avg_stiffness_mpa']:<12.1f} | {r['total_damage']:<14.5e} | {lane_factor:<12.2f} | {weighted_damage:<14.5e}")
    
    direction_results[direction] = {
        'weighted_damage': dir_weighted_damage,
        'weighted_damage_no_healing': dir_weighted_damage_no_healing,
        'total_vehicles': dir_vehicles
    }
    
    print(f"  >> {direction} Subtotal: {dir_vehicles} vehicles, Weighted Damage: {dir_weighted_damage:.5e}")
    print("-" * 130)

# Total vehicles
total_vehicles = sum(r['total_vehicles'] for r in lane_results.values())

print(f"\n{'='*130}")
print("DAMAGE SUMMARY BY ROAD DIRECTION")
print("="*130)


for direction, dr in direction_results.items():
    print(f"\n--- {direction} ---")
    print(f"Total Vehicle Passes:        {dr['total_vehicles']}")
    print(f"Damage WITHOUT healing:      {dr['weighted_damage_no_healing']:.6e}")
    print(f"Damage WITH healing:         {dr['weighted_damage']:.6e}")
    
    if dr['weighted_damage_no_healing'] > 0:
        healing_benefit = ((dr['weighted_damage_no_healing'] - dr['weighted_damage']) / dr['weighted_damage_no_healing'] * 100)
        print(f"Healing Benefit:             {healing_benefit:.2f}% reduction")
    
    if dr['weighted_damage'] > 0:
        remaining_life = 1.0 / dr['weighted_damage']
        print(f"Remaining Life:              {remaining_life:.3e} simulation periods ({remaining_life/8760:.1f} years)")

# Find critical direction (highest damage)
critical_direction = max(direction_results.items(), key=lambda x: x[1]['weighted_damage'])

print(f"\n{'='*130}")
print("CRITICAL DIRECTION ASSESSMENT")
print("="*130)
print(f"\nCritical Direction: {critical_direction[0]}")
print(f"Maximum Weighted Damage: {critical_direction[1]['weighted_damage']:.6e}")
if critical_direction[1]['weighted_damage'] > 0:
    critical_life = 1.0 / critical_direction[1]['weighted_damage']
    print(f"Pavement Life (Critical): {critical_life:.3e} simulation periods")
    print(f"                          ({critical_life/8760:.1f} years at current traffic rate)")
