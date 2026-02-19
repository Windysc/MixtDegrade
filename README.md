# Pavement Fatigue Analysis (MLET-based)

This project provides a Python-based tool (`pavement_analysis.py`) for analyzing pavement fatigue damage using traffic data from SUMO (Simulation of Urban MObility). It employs Multi-Layer Elastic Theory (MLET) to calculate strain and estimates fatigue life considering vehicle axle configurations, dynamic modulus of asphalt, and healing effects due to rest periods.

## Key Features

*   **MLET-based Strain Calculation**: Calculates tensile strain at the bottom of the asphalt layer using Multi-Layer Elastic Theory, considering layer properties and load characteristics.
*   **Multi-Axle Support**: detailed handling of single, tandem, and tridem axle groups, including load splitting and superposition effects for strain calculation.
*   **Dynamic Modulus**: Calculates asphalt dynamic modulus ($|E^*|$) based on vehicle speed (loading frequency) and temperature using a Sigmoidal Master Curve and WLF shift factors.
*   **SMHI Temperature Integration**: Fetches real-time or historical temperature data from SMHI (Swedish Meteorological and Hydrological Institute) Open Data API with multiple modes (CONSTANT, REALTIME, DAILY, CACHED).
*   **Healing Effect**: Incorporates a fatigue model that accounts for rest periods between vehicle loads, quantifying the "healing" or recovery benefit of intermittent loading.
*   **Detailed Vehicle Modeling**: Defines specific axle configurations (weights, types) for various vehicle classes (Passenger, Truck, HGV, Bus, etc.).
*   **Lane-by-Lane Analysis**: Processes SUMO XML lane output files individually and aggregates results by road direction.
*   **Odemark's Equivalent Thickness**: Transforms multi-layer pavement into equivalent two-layer system using Swedish Odemark method (1949).
*   **Lane Change Visualization**: Generates spatial and temporal visualizations of lane change behavior, including single vehicle trajectory tracking.

## Prerequisites

*   Python 3.x
*   Standard libraries: `xml.etree.ElementTree`, `math`, `os`, `glob`, `json`, `urllib`, `datetime`
*   Visualization: `pandas`, `matplotlib`, `seaborn`, `numpy`

## Usage

1.  **Prepare Data**: Ensure your SUMO simulation output files (XML format, e.g., `e1i_0.xml`) are in the same directory as the script.
2.  **Configuration**: Configure temperature model mode (`TEMP_MODEL_MODE`) and SMHI station ID (`SMHI_STATION_ID`) in the script. Modes:
    - `CONSTANT`: Use fixed temperature value
    - `REALTIME`: Fetch hourly data from SMHI API
    - `DAILY`: Use daily mean temperature from SMHI API
    - `CACHED`: Use previously cached data (offline mode)
3.  **Run the Script**:
    ```bash
    python pavement_analysis.py
    ```
4.  **View Results**: The script outputs detailed analysis to the console, including damage contributions and remaining life estimates.

5.  **Lane Change Visualization**: Run `visualize_lane.py` to generate lane change behavior visualizations:
    ```bash
    python visualize_lane.py
    ```
    Produces 6 visualizations:
    - Lane changes by road segment (top 20 edges)
    - Normalized merging behavior distribution
    - Physical map of lane change hotspots
    - Lane change frequency over time
    - Overtaking vs. returning lane transitions
    - Single vehicle activity analysis (interactive selection)
   

## Calculation Process and Parameters

This section details the calculation logic and parameters used in the script.

### 1. Parameters

#### Pavement Structure
The pavement is modeled as a Swedish multi-layer system using Odemark's equivalent thickness method:

| Layer | Material | Thickness (mm) | Modulus (MPa) | Poisson's Ratio |
| :--- | :--- | :--- | :--- | :--- |
| **Wearing** | ABT11 70/100 | 40 | 5500 | 0.35 |
| **Binder** | ABb16 50/70 | 50 | 6500 | 0.35 |
| **Bound Base** | AG22 160/220 | 65 | 4000 | 0.35 |
| **Unbound Base** | GW-CR (4-6% fines) | 80 | 300 | 0.35 |
| **Subbase** | GW-CR (4-6% fines) | 420 | 200 | 0.35 |
| **Subgrade** | 4e-Lera (Clay) | Infinite | 20 | 0.45 |

*Note: Bitumen-bound layer moduli are temperature-dependent using master curve.*

#### Vehicle Configuration
Vehicles are defined by their total weight and axle configurations.

| Vehicle Type | Total Weight (kN) | Axle Configuration |
| :--- | :--- | :--- |
| **Passenger** | 15 | Front: 7kN (Single), Rear: 8kN (Single) |
| **Truck** | 180 | Front: 60kN (Single), Rear: 120kN (Tandem) |
| **HGV** | 400 | Front: 70kN (Single), Drive: 180kN (Tandem), Trailer: 150kN (Tandem) |
| **Bus** | 180 | Front: 70kN (Single), Rear: 110kN (Single) |
| **Trailer** | 440 | Front: 70kN (Single), Drive: 180kN (Tandem), Trailer: 190kN (Tridem) |
| **Autonomous**| 18 | Front: 7kN (Single), Rear: 8kN (Single) |

#### Fatigue Model Parameters
The fatigue life ($N_f$) is calculated using the following model:
$N_f = a \cdot (1/\epsilon)^b \cdot (1/E)^c \cdot \exp(d \cdot T + f \cdot RP)$

Where coefficients depend on the axle type, based on the findings of (H. Cheng et al, [2023](https://www.nature.com/articles/s41467-023-40116-0))

| Axle Type | a | b | c | d | f |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Single** | 8.593e16 | 2.769 | 1.230 | 0.046 | 1.062 |
| **Tandem** | 2.823e14 | 2.353 | 0.974 | 0.043 | 1.360 |
| **Tridem** | 8.891e13 | 2.051 | 1.115 | 0.042 | 1.310 |

### 2. Core Calculation Functions

The calculation follows a step-by-step process for each vehicle passage.

#### A. Dynamic Modulus ($|E^*|$)
Calculates the stiffness of the asphalt layer based on vehicle speed (which determines loading frequency) and temperature.

```python
def calculate_asphalt_modulus(speed_ms, temp_c):
    # 1. Calculate loading frequency: f = v / (2 * a)
    freq = calculate_loading_frequency(speed_ms)
    
    # 2. Calculate WLF temperature shift factor
    a_t = calculate_wlf_shift(temp_c)
    
    # 3. Calculate reduced frequency
    freq_r = freq * a_t
    
    # 4. Use Sigmoidal Master Curve to get log(E)
    log_fr = math.log10(freq_r)
    log_E = SIG_DELTA + SIG_ALPHA / (1.0 + math.exp(SIG_BETA + SIG_GAMMA * log_fr))
    
    return 10 ** log_E
```

#### B. Strain Calculation (MLET)
Calculates the tensile strain at the bottom of the asphalt layer. For multi-axle groups (tandem/tridem), it accounts for the superposition of strains from adjacent axles.

```python
def mlet_strain_calculation(axle_load_kn, E_asphalt, speed_ms, axle_type='single'):
    # ... (Setup geometry and layer properties) ...
    
    # Calculate strain factor (F_epsilon) based on MLET charts
    # Function of contact radius (a/h1) and modular ratio (E1/E2)
    F_epsilon = F_base * F_modular
    
    # Calculate single axle strain
    epsilon_single = (1.0 + NU_ASPHALT) * (q / E1) * F_epsilon
    
    # Handle Multi-Axle Superposition
    if num_axles > 1:
        # Calculate influence of adjacent axles based on spacing
        superposition_sum = 1.0
        for i in range(1, num_axles):
            r = i * spacing
            influence = 1.0 / ((1.0 + (r/h1)**2) ** 1.5) # Boussinesq-type decay
            superposition_sum += influence
        
        epsilon_t = epsilon_single * superposition_sum
    else:
        epsilon_t = epsilon_single
        
    return epsilon_t
```

#### C. Fatigue Life ($N_f$)
Calculates the number of load repetitions to failure for a specific strain level, taking into account the healing effect of rest periods.

```python
def calculate_fatigue_life(strain, E_asphalt, temp_c, rest_period_seconds, axle_type='single'):
    # Select parameters (a, b, c, d, f) based on axle_type
    # ...
    
    # Apply Fatigue Equation
    strain_term = strain_microstrain ** (-b)
    modulus_term = E_asphalt ** (-c)
    temp_rest_term = math.exp(d * temp_c + f * rest_period_seconds)
    
    N_f = a * strain_term * modulus_term * temp_rest_term * SHIFT_FACTOR
    
    return max(N_f, 1.0)
```

### 3. Calculation Workflow

1.  **Parse XML**: Read SUMO lane output files to get vehicle events (ID, type, speed, time).
2.  **Iterate Vehicles**: For each vehicle:
    *   Determine its axle configuration (e.g., Truck -> Single Front + Tandem Rear).
    *   Calculate the time gap (rest period) since the *previous* vehicle.
    *   Calculate Asphalt Modulus ($|E^*|$) using current speed and temperature.
3.  **Iterate Axles**: For each axle group on the vehicle:
    *   **Check Threshold**: Ignore axles below `MINIMUM_DAMAGING_AXLE_LOAD` (1.0 kN).
    *   **Calculate Strain**: Use `mlet_strain_calculation` to get $\epsilon_t$.
    *   **Calculate $N_f$**: Use `calculate_fatigue_life` with the axle's strain and rest period.
    *   **Accumulate Damage**: Use Miner's Rule ($D = \sum 1/N_f$) to add damage.
4.  **Aggregation**:
    *   Sum damage for all vehicles in a lane.
    *   Apply `LANE_FACTORS` to account for lateral wander.
    *   Sum weighted damage across all lanes for a road direction.
    *   Compute remaining life ($1 / D_{total}$).
