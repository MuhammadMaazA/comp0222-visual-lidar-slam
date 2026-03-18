#!/usr/bin/env python3
"""
Enhanced Part 1 Analysis Script
Creates comprehensive analysis and visualization for ORB-SLAM2 parameter experiments
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import zipfile
import re
from pathlib import Path
import json

class Part1Analyzer:
    def __init__(self, results_dir="results"):
        self.results_dir = Path(results_dir)
        self.results = {}
        
    def extract_evo_results(self):
        """Extract numerical results from evo zip files"""
        results = []
        
        zip_files = list(self.results_dir.glob("*.zip"))
        print(f"📁 Found {len(zip_files)} result files")
        
        for zip_file in zip_files:
            try:
                # Parse filename to extract experiment info
                name_parts = zip_file.stem.split('_')
                dataset = name_parts[0]  # tum or kitti07
                task = '_'.join(name_parts[1:])  # 1a_baseline, 1b_feat1500, etc.
                
                # Extract metrics from zip file
                with zipfile.ZipFile(zip_file, 'r') as zf:
                    file_list = zf.namelist()
                    
                    # Look for stats.json or similar
                    stats_file = None
                    for f in file_list:
                        if 'stats' in f.lower() and f.endswith('.json'):
                            stats_file = f
                            break
                    
                    if stats_file:
                        with zf.open(stats_file) as f:
                            stats = json.load(f)
                            
                        # Extract key metrics
                        result = {
                            'dataset': dataset,
                            'experiment': task,
                            'rmse': stats.get('rmse', None),
                            'mean': stats.get('mean', None),
                            'median': stats.get('median', None),
                            'std': stats.get('std', None),
                            'min': stats.get('min', None),
                            'max': stats.get('max', None),
                            'sse': stats.get('sse', None)
                        }
                        results.append(result)
                        print(f"✅ Extracted: {dataset} - {task} (RMSE: {result['rmse']:.3f})")
                    
            except Exception as e:
                print(f"⚠️ Error processing {zip_file}: {e}")
                continue
        
        return pd.DataFrame(results)
    
    def create_comparison_table(self, df):
        """Create comprehensive comparison table"""
        print("\n📊 === PART 1: ORB-SLAM2 PARAMETER ANALYSIS RESULTS ===")
        print("="*80)
        
        # Separate by dataset
        tum_results = df[df['dataset'] == 'tum'].copy()
        kitti_results = df[df['dataset'] == 'kitti07'].copy()
        
        if not tum_results.empty:
            print("\n🎯 TUM DATASET RESULTS:")
            print("-" * 50)
            tum_display = tum_results[['experiment', 'rmse', 'mean', 'median', 'std']].round(4)
            print(tum_display.to_string(index=False))
            
        if not kitti_results.empty:
            print("\n🚗 KITTI DATASET RESULTS:")
            print("-" * 50)
            kitti_display = kitti_results[['experiment', 'rmse', 'mean', 'median', 'std']].round(4)
            print(kitti_display.to_string(index=False))
        
        return tum_results, kitti_results
    
    def create_parameter_analysis(self, tum_df, kitti_df):
        """Analyze parameter effects"""
        print("\n🔬 === PARAMETER EFFECT ANALYSIS ===")
        print("="*60)
        
        # Define baseline and experiments
        experiments = {
            '1a_baseline': 'Baseline (1000 features)',
            '1b_feat1500': 'High Features (1500)',
            '1b_feat800': 'Low Features (800)', 
            '1c_nooutlier': 'No Outlier Rejection',
            '1d_noloop': 'No Loop Closure'
        }
        
        for dataset_name, df in [("TUM", tum_df), ("KITTI", kitti_df)]:
            if df.empty:
                continue
                
            print(f"\n📈 {dataset_name} Parameter Effects:")
            print("-" * 40)
            
            baseline_rmse = None
            if '1a_baseline' in df['experiment'].values:
                baseline_rmse = df[df['experiment'] == '1a_baseline']['rmse'].iloc[0]
                print(f"Baseline RMSE: {baseline_rmse:.4f}")
                
            for exp, desc in experiments.items():
                if exp in df['experiment'].values:
                    rmse = df[df['experiment'] == exp]['rmse'].iloc[0]
                    if baseline_rmse and exp != '1a_baseline':
                        change = ((rmse - baseline_rmse) / baseline_rmse) * 100
                        direction = "↑" if change > 0 else "↓"
                        print(f"{desc:25} | RMSE: {rmse:.4f} | Change: {direction}{abs(change):5.1f}%")
                    elif exp == '1a_baseline':
                        print(f"{desc:25} | RMSE: {rmse:.4f} | (Baseline)")
    
    def create_enhanced_visualizations(self, tum_df, kitti_df):
        """Create comprehensive visualization plots"""
        fig = plt.figure(figsize=(20, 12))
        
        # 1. RMSE Comparison Bar Plot
        plt.subplot(2, 3, 1)
        datasets = []
        rmses = []
        experiments = []
        
        for name, df in [("TUM", tum_df), ("KITTI", kitti_df)]:
            if not df.empty:
                for _, row in df.iterrows():
                    datasets.append(name)
                    rmses.append(row['rmse'])
                    experiments.append(row['experiment'])
        
        if datasets:
            x_pos = np.arange(len(experiments))
            colors = ['blue' if d == 'TUM' else 'orange' for d in datasets]
            bars = plt.bar(x_pos, rmses, color=colors, alpha=0.7)
            plt.xlabel('Experiment')
            plt.ylabel('RMSE (m)')
            plt.title('Part 1: RMSE Comparison Across Experiments')
            plt.xticks(x_pos, [e.replace('_', '\n') for e in experiments], rotation=45)
            plt.grid(True, alpha=0.3)
            
            # Add value labels on bars
            for i, (bar, rmse) in enumerate(zip(bars, rmses)):
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                        f'{rmse:.3f}', ha='center', va='bottom', fontsize=8)
        
        # 2. Parameter Effect Analysis (TUM)
        plt.subplot(2, 3, 2)
        if not tum_df.empty and '1a_baseline' in tum_df['experiment'].values:
            baseline = tum_df[tum_df['experiment'] == '1a_baseline']['rmse'].iloc[0]
            exp_names = []
            changes = []
            
            for _, row in tum_df.iterrows():
                if row['experiment'] != '1a_baseline':
                    exp_names.append(row['experiment'].replace('_', '\n'))
                    change = ((row['rmse'] - baseline) / baseline) * 100
                    changes.append(change)
            
            if changes:
                colors = ['red' if c > 0 else 'green' for c in changes]
                plt.bar(exp_names, changes, color=colors, alpha=0.7)
                plt.ylabel('% Change from Baseline')
                plt.title('TUM: Parameter Effect on RMSE')
                plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
                plt.grid(True, alpha=0.3)
                plt.xticks(rotation=45)
        
        # 3. Parameter Effect Analysis (KITTI)
        plt.subplot(2, 3, 3)
        if not kitti_df.empty and '1a_baseline' in kitti_df['experiment'].values:
            baseline = kitti_df[kitti_df['experiment'] == '1a_baseline']['rmse'].iloc[0]
            exp_names = []
            changes = []
            
            for _, row in kitti_df.iterrows():
                if row['experiment'] != '1a_baseline':
                    exp_names.append(row['experiment'].replace('_', '\n'))
                    change = ((row['rmse'] - baseline) / baseline) * 100
                    changes.append(change)
            
            if changes:
                colors = ['red' if c > 0 else 'green' for c in changes]
                plt.bar(exp_names, changes, color=colors, alpha=0.7)
                plt.ylabel('% Change from Baseline')
                plt.title('KITTI: Parameter Effect on RMSE')
                plt.axhline(y=0, color='black', linestyle='-', alpha=0.5)
                plt.grid(True, alpha=0.3)
                plt.xticks(rotation=45)
        
        # 4. Statistical Distribution Comparison
        plt.subplot(2, 3, 4)
        metrics = ['rmse', 'mean', 'std']
        x_pos = np.arange(len(metrics))
        width = 0.35
        
        if not tum_df.empty and not kitti_df.empty:
            # Use baseline values for comparison
            tum_baseline = tum_df[tum_df['experiment'] == '1a_baseline'].iloc[0] if '1a_baseline' in tum_df['experiment'].values else tum_df.iloc[0]
            kitti_baseline = kitti_df[kitti_df['experiment'] == '1a_baseline'].iloc[0] if '1a_baseline' in kitti_df['experiment'].values else kitti_df.iloc[0]
            
            tum_values = [tum_baseline[m] for m in metrics]
            kitti_values = [kitti_baseline[m] for m in metrics]
            
            plt.bar(x_pos - width/2, tum_values, width, label='TUM', alpha=0.7)
            plt.bar(x_pos + width/2, kitti_values, width, label='KITTI', alpha=0.7)
            plt.xlabel('Metric')
            plt.ylabel('Value (m)')
            plt.title('Baseline Performance Comparison')
            plt.xticks(x_pos, ['RMSE', 'Mean', 'Std Dev'])
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        # 5. Feature Count Analysis
        plt.subplot(2, 3, 5)
        feature_experiments = ['1a_baseline', '1b_feat800', '1b_feat1500']
        feature_counts = [1000, 800, 1500]
        
        tum_rmses = []
        kitti_rmses = []
        tum_features = []
        kitti_features = []
        
        for i, exp in enumerate(feature_experiments):
            tum_val = tum_df[tum_df['experiment'] == exp]['rmse'].iloc[0] if not tum_df.empty and exp in tum_df['experiment'].values else None
            kitti_val = kitti_df[kitti_df['experiment'] == exp]['rmse'].iloc[0] if not kitti_df.empty and exp in kitti_df['experiment'].values else None
            
            if tum_val is not None:
                tum_rmses.append(tum_val)
                tum_features.append(feature_counts[i])
            if kitti_val is not None:
                kitti_rmses.append(kitti_val)
                kitti_features.append(feature_counts[i])
        
        if tum_rmses:
            plt.plot(tum_features, tum_rmses, 'bo-', label='TUM', linewidth=2, markersize=8)
        if kitti_rmses:
            plt.plot(kitti_features, kitti_rmses, 'ro-', label='KITTI', linewidth=2, markersize=8)
        
        if tum_rmses or kitti_rmses:
            plt.xlabel('Number of ORB Features')
            plt.ylabel('RMSE (m)')
            plt.title('Feature Count vs Accuracy')
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        # 6. Component Analysis
        plt.subplot(2, 3, 6)
        components = ['Baseline', 'No Outliers', 'No Loop Closure']
        component_exps = ['1a_baseline', '1c_nooutlier', '1d_noloop']
        
        tum_vals = []
        kitti_vals = []
        
        for exp in component_exps:
            tum_val = tum_df[tum_df['experiment'] == exp]['rmse'].iloc[0] if not tum_df.empty and exp in tum_df['experiment'].values else 0
            kitti_val = kitti_df[kitti_df['experiment'] == exp]['rmse'].iloc[0] if not kitti_df.empty and exp in kitti_df['experiment'].values else 0
            tum_vals.append(tum_val)
            kitti_vals.append(kitti_val)
        
        x_pos = np.arange(len(components))
        width = 0.35
        
        if any(tum_vals) or any(kitti_vals):
            plt.bar(x_pos - width/2, tum_vals, width, label='TUM', alpha=0.7)
            plt.bar(x_pos + width/2, kitti_vals, width, label='KITTI', alpha=0.7)
            plt.xlabel('SLAM Component')
            plt.ylabel('RMSE (m)')
            plt.title('Component Contribution Analysis')
            plt.xticks(x_pos, components, rotation=15)
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('part1_enhanced_analysis.png', dpi=300, bbox_inches='tight')
        print(f"\n📊 Enhanced visualization saved as 'part1_enhanced_analysis.png'")
        
        return fig
    
    def generate_comprehensive_report(self, df, tum_df, kitti_df):
        """Generate detailed text report"""
        report = []
        report.append("# Part 1: Enhanced ORB-SLAM2 Parameter Analysis Report")
        report.append("=" * 70)
        report.append("")
        
        report.append("## 🎯 Executive Summary")
        report.append("")
        report.append(f"- **Total Experiments**: {len(df)}")
        report.append(f"- **TUM Results**: {len(tum_df)} experiments")
        report.append(f"- **KITTI Results**: {len(kitti_df)} experiments")
        report.append("")
        
        # Best/worst performance
        if not df.empty:
            best_idx = df['rmse'].idxmin()
            worst_idx = df['rmse'].idxmax()
            best = df.loc[best_idx]
            worst = df.loc[worst_idx]
            
            report.append(f"- **Best Performance**: {best['dataset']} - {best['experiment']} (RMSE: {best['rmse']:.4f}m)")
            report.append(f"- **Worst Performance**: {worst['dataset']} - {worst['experiment']} (RMSE: {worst['rmse']:.4f}m)")
            report.append("")
        
        report.append("## 📊 Detailed Results")
        report.append("")
        
        # TUM Analysis
        if not tum_df.empty:
            report.append("### 🎯 TUM Dataset Analysis")
            report.append("")
            for _, row in tum_df.iterrows():
                report.append(f"**{row['experiment']}**: RMSE={row['rmse']:.4f}m, Mean={row['mean']:.4f}m, Std={row['std']:.4f}m")
            report.append("")
            
            # Parameter insights
            if '1a_baseline' in tum_df['experiment'].values:
                baseline = tum_df[tum_df['experiment'] == '1a_baseline']['rmse'].iloc[0]
                report.append("**Parameter Effects (vs Baseline):**")
                for _, row in tum_df.iterrows():
                    if row['experiment'] != '1a_baseline':
                        change = ((row['rmse'] - baseline) / baseline) * 100
                        direction = "worse" if change > 0 else "better"
                        report.append(f"- {row['experiment']}: {abs(change):.1f}% {direction}")
                report.append("")
        
        # KITTI Analysis  
        if not kitti_df.empty:
            report.append("### 🚗 KITTI Dataset Analysis")
            report.append("")
            for _, row in kitti_df.iterrows():
                report.append(f"**{row['experiment']}**: RMSE={row['rmse']:.4f}m, Mean={row['mean']:.4f}m, Std={row['std']:.4f}m")
            report.append("")
            
            # Parameter insights
            if '1a_baseline' in kitti_df['experiment'].values:
                baseline = kitti_df[kitti_df['experiment'] == '1a_baseline']['rmse'].iloc[0]
                report.append("**Parameter Effects (vs Baseline):**")
                for _, row in kitti_df.iterrows():
                    if row['experiment'] != '1a_baseline':
                        change = ((row['rmse'] - baseline) / baseline) * 100
                        direction = "worse" if change > 0 else "better"
                        report.append(f"- {row['experiment']}: {abs(change):.1f}% {direction}")
                report.append("")
        
        report.append("## 🔬 Key Insights")
        report.append("")
        report.append("### Feature Count Analysis")
        report.append("- Higher feature counts (1500) may improve or degrade performance depending on scene")
        report.append("- Lower feature counts (800) generally reduce computational cost")
        report.append("")
        report.append("### Component Analysis")
        report.append("- Outlier rejection is critical for robust performance")
        report.append("- Loop closure helps with drift correction over longer sequences")
        report.append("")
        report.append("## 📁 Generated Files")
        report.append("")
        report.append("- `part1_enhanced_analysis.png` - Comprehensive visualization plots")
        report.append("- `part1_detailed_results.csv` - Numerical results table")
        report.append("- `part1_analysis_report.txt` - This detailed report")
        
        # Save report
        with open('part1_analysis_report.txt', 'w') as f:
            f.write('\n'.join(report))
        
        print(f"\n📋 Detailed report saved as 'part1_analysis_report.txt'")
        return '\n'.join(report)

def main():
    print("🚀 Starting Enhanced Part 1 Analysis")
    print("=" * 50)
    
    analyzer = Part1Analyzer()
    
    # Extract results from evo zip files
    df = analyzer.extract_evo_results()
    
    if df.empty:
        print("❌ No results found. Please ensure evo result zip files are in the 'results' directory.")
        return
    
    # Save raw results
    df.to_csv('part1_detailed_results.csv', index=False)
    print(f"💾 Raw results saved to 'part1_detailed_results.csv'")
    
    # Create comparison tables
    tum_df, kitti_df = analyzer.create_comparison_table(df)
    
    # Parameter analysis
    analyzer.create_parameter_analysis(tum_df, kitti_df)
    
    # Create visualizations
    analyzer.create_enhanced_visualizations(tum_df, kitti_df)
    
    # Generate comprehensive report
    analyzer.generate_comprehensive_report(df, tum_df, kitti_df)
    
    print("\n✅ Enhanced Part 1 analysis completed successfully!")
    print("📊 Check generated files:")
    print("  - part1_enhanced_analysis.png")
    print("  - part1_detailed_results.csv") 
    print("  - part1_analysis_report.txt")

if __name__ == "__main__":
    main()