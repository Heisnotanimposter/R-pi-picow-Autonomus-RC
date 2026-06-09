#!/usr/bin/env python3
"""
Waymo / nuScenes Exploratory Data Analysis (EDA) & Data Cleaning Script.
Implements data loading, confidence filtering, range thresholding, 
and Bird's-Eye-View (BEV) spatial scatter plotting using Pandas.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

def generate_mock_sensor_data(num_samples=150):
    """
    Generates a mock dataset resembling raw nuScenes/Waymo 3D bounding box records
    with added noise, outliers, and null values for cleaning demonstration.
    """
    np.random.seed(42)
    
    # Common classes in nuScenes
    classes = ['car', 'pedestrian', 'truck', 'bicycle', 'barrier', 'traffic_cone', 'unknown']
    class_probs = [0.45, 0.20, 0.10, 0.05, 0.10, 0.08, 0.02]
    
    data = {
        'object_id': [f"obj_{i:04d}" for i in range(num_samples)],
        'class_name': np.random.choice(classes, size=num_samples, p=class_probs),
        # Positions relative to ego-vehicle (meters)
        'pos_x': np.random.uniform(-75, 75, size=num_samples),
        'pos_y': np.random.uniform(-75, 75, size=num_samples),
        'pos_z': np.random.uniform(-2, 3, size=num_samples),
        # Dimension fields (length, width, height in meters)
        'dim_l': np.random.normal(4.2, 0.5, size=num_samples),
        'dim_w': np.random.normal(1.8, 0.2, size=num_samples),
        'dim_h': np.random.normal(1.5, 0.3, size=num_samples),
        # Orientation (yaw in radians)
        'yaw': np.random.uniform(-np.pi, np.pi, size=num_samples),
        # Velocity vector
        'vel_x': np.random.normal(0, 8, size=num_samples),
        'vel_y': np.random.normal(0, 8, size=num_samples),
        # Detection Confidence score (0.0 to 1.0)
        'confidence': np.random.uniform(0.1, 1.0, size=num_samples)
    }
    
    df = pd.DataFrame(data)
    
    # Introduce anomalies for cleaning:
    # 1. Null values in position/confidence
    df.loc[np.random.choice(df.index, 5, replace=False), 'pos_x'] = np.nan
    df.loc[np.random.choice(df.index, 5, replace=False), 'confidence'] = np.nan
    
    # 2. Physics outliers (impossible velocities/dimensions)
    df.loc[12, 'vel_x'] = 450.0  # Speeding rocket! (1620 km/h)
    df.loc[25, 'dim_l'] = 95.0   # 95 meters long vehicle
    df.loc[45, 'pos_y'] = 999.0  # Point far out of bounds
    
    return df

def clean_sensor_data(df):
    """
    Cleans raw object detection records using Pandas:
    - Drops records with missing essential variables (pos_x, confidence).
    - Filters out low confidence detections (confidence < 0.4).
    - Removes physical anomalies/outliers (speeds > 150 m/s, lengths > 25m, range > 100m).
    - Calculates distance to Ego-vehicle.
    """
    print("\n--- Raw Data Summary ---")
    print(f"Total raw detection instances: {len(df)}")
    print(df.info())
    print("\nMissing values count:")
    print(df.isnull().sum())
    
    # 1. Drop rows with null positions or null confidence scores
    cleaned_df = df.dropna(subset=['pos_x', 'pos_y', 'confidence']).copy()
    
    # 2. Filter out low-confidence detections (potential sensor noise)
    min_confidence = 0.40
    cleaned_df = cleaned_df[cleaned_df['confidence'] >= min_confidence]
    
    # 3. Calculate distance to ego-vehicle (ego vehicle is at origin x=0, y=0, z=0)
    cleaned_df['distance'] = np.sqrt(cleaned_df['pos_x']**2 + cleaned_df['pos_y']**2 + cleaned_df['pos_z']**2)
    
    # 4. Remove physical outliers
    # Max speed filter (e.g. 120 m/s or 432 km/h)
    cleaned_df['speed'] = np.sqrt(cleaned_df['vel_x']**2 + cleaned_df['vel_y']**2)
    cleaned_df = cleaned_df[cleaned_df['speed'] < 120.0]
    
    # Maximum vehicle dimensions
    cleaned_df = cleaned_df[cleaned_df['dim_l'] < 25.0]
    
    # Limit operational range to 80 meters (lidar/radar envelope)
    cleaned_df = cleaned_df[cleaned_df['distance'] <= 80.0]
    
    # Remove 'unknown' class classifications
    cleaned_df = cleaned_df[cleaned_df['class_name'] != 'unknown']
    
    print("\n--- Cleaned Data Summary ---")
    print(f"Total valid detection instances: {len(cleaned_df)}")
    print(cleaned_df.describe())
    
    return cleaned_df

def plot_bev_visualizations(df, output_path='eda_bev_plot.png'):
    """
    Creates exploratory data plots:
    1. Histogram showing count distribution of detected object classes.
    2. Bird's-Eye-View (BEV) scatter plot of surroundings relative to ego-vehicle.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: Class Distribution Bar Chart
    class_counts = df['class_name'].value_counts()
    colors = ['#3b82f6', '#10b981', '#f59e0b', '#ef4444', '#8b5cf6', '#6b7280']
    axes[0].bar(class_counts.index, class_counts.values, color=colors[:len(class_counts)])
    axes[0].set_title("Detected Object Class Distribution", fontsize=12, fontweight='bold')
    axes[0].set_xlabel("Object Classification Label")
    axes[0].set_ylabel("Occurrences")
    axes[0].grid(axis='y', linestyle='--', alpha=0.5)
    
    # Plot 2: Bird's-Eye-View (BEV) Scatter Plot
    # Ego vehicle is located at (0, 0)
    axes[1].scatter(0, 0, color='red', marker='^', s=150, label='Ego Vehicle')
    
    # Draw radar concentric distance rings
    for r in [20, 40, 60, 80]:
        circle = plt.Circle((0, 0), r, color='#64748b', fill=False, linestyle=':', alpha=0.5)
        axes[1].add_patch(circle)
        axes[1].text(0, r + 1, f"{r}m", color='#64748b', fontsize=8, ha='center')

    # Scatter other objects by category
    categories = df['class_name'].unique()
    for cat in categories:
        cat_df = df[df['class_name'] == cat]
        axes[1].scatter(
            cat_df['pos_x'], 
            cat_df['pos_y'], 
            alpha=0.8, 
            s=cat_df['dim_l']*cat_df['dim_w']*10, # scale dot size with object volume
            label=f"{cat.capitalize()}"
        )
        
    axes[1].set_title("Bird's-Eye-View (BEV) Obstacle Scatter Map", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("Lateral Distance (X, meters)")
    axes[1].set_ylabel("Longitudinal Distance (Y, meters)")
    axes[1].set_xlim(-90, 90)
    axes[1].set_ylim(-90, 90)
    axes[1].grid(True, linestyle='--', alpha=0.3)
    axes[1].legend(loc='upper right')
    axes[1].set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\n[EDA] Saved visual summary plot to {output_path}")
    plt.close()

if __name__ == '__main__':
    # Execute full cleaning and analysis process
    print("====================================================")
    print("nuScenes/Waymo Dataset Cleaning & EDA Pipeline")
    print("====================================================")
    
    # Load raw dataset
    raw_df = generate_mock_sensor_data()
    
    # Clean anomalies and filter noise
    clean_df = clean_sensor_data(raw_df)
    
    # Export cleaned dataset to CSV
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_out = os.path.join(script_dir, 'cleaned_object_detections.csv')
    clean_df.to_csv(csv_out, index=False)
    print(f"[EDA] Exported cleaned dataset to: {csv_out}")
    
    # Generate charts
    img_out = os.path.join(script_dir, 'eda_bev_plot.png')
    plot_bev_visualizations(clean_df, output_path=img_out)
    
    # Aggregate statistics
    print("\n--- Average Object Dimensions ---")
    print(clean_df.groupby('class_name')[['dim_l', 'dim_w', 'dim_h']].mean())
