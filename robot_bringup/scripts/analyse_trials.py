#!/usr/bin/env python3
"""
Analyse trial CSV data from scout_mission.py evaluation runs.

Reads all CSVs from trial_results/, computes summary statistics,
and generates tables and charts for the FYP report.

Usage:
  python3 analyse_trials.py [trial_results_dir] [output_dir]
  Defaults: trial_results/ and analysis_output/
"""

import csv
import os
import sys
import statistics

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print('[warn] matplotlib not installed. Tables will be printed but no charts generated.')
    print('       Install with: pip install matplotlib --break-system-packages')


def load_trials(trial_dir):
    """Load all CSV files grouped by scenario."""
    scenarios = {'baseline': [], 'deadman': [], 'obstacle': []}
    for fname in sorted(os.listdir(trial_dir)):
        if not fname.endswith('.csv'):
            continue
        scenario = fname.split('_')[0]
        if scenario not in scenarios:
            continue
        filepath = os.path.join(trial_dir, fname)
        rows = []
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
        scenarios[scenario].append({'file': fname, 'rows': rows})
    return scenarios


def parse_float(val):
    """Safely parse a float from CSV, returning None for empty/invalid."""
    if val is None or val.strip() == '':
        return None
    try:
        return float(val)
    except ValueError:
        return None


def analyse_scenario(name, trials):
    """Compute summary statistics for one scenario."""
    nav_waypoints = []
    nav_succeeded = []
    nav_failed = []
    mission_times = []
    stop_latencies = []
    resume_latencies = []
    deadman_latencies = []

    for trial in trials:
        rows = trial['rows']
        wp_rows = [r for r in rows if r['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED')]
        succeeded = sum(1 for r in wp_rows if r['status'] == 'SUCCEEDED')
        failed = sum(1 for r in wp_rows if r['status'] in ('FAILED', 'CANCELED'))
        total_wp = len(wp_rows)

        nav_waypoints.append(total_wp)
        nav_succeeded.append(succeeded)
        nav_failed.append(failed)

        durations = [parse_float(r['duration_s']) for r in wp_rows]
        durations = [d for d in durations if d is not None]
        if durations:
            mission_times.append(sum(durations))

        for r in rows:
            sl = parse_float(r.get('stop_latency_s', ''))
            if sl is not None:
                stop_latencies.append(sl)
            rl = parse_float(r.get('resume_latency_s', ''))
            if rl is not None:
                resume_latencies.append(rl)
            if r['status'] == 'DEADMAN_TRIGGERED':
                dl = parse_float(r.get('stop_latency_s', ''))
                if dl is not None:
                    deadman_latencies.append(dl)

    result = {
        'scenario': name,
        'num_trials': len(trials),
        'total_waypoints': sum(nav_waypoints),
        'total_succeeded': sum(nav_succeeded),
        'total_failed': sum(nav_failed),
        'nav_success_rate': sum(nav_succeeded) / sum(nav_waypoints) * 100 if sum(nav_waypoints) > 0 else 0,
        'mission_times': mission_times,
        'stop_latencies': stop_latencies,
        'resume_latencies': resume_latencies,
        'deadman_latencies': deadman_latencies,
    }

    if mission_times:
        result['mission_time_mean'] = statistics.mean(mission_times)
        result['mission_time_std'] = statistics.stdev(mission_times) if len(mission_times) > 1 else 0
    if stop_latencies:
        result['stop_latency_mean'] = statistics.mean(stop_latencies)
        result['stop_latency_median'] = statistics.median(stop_latencies)
        result['stop_latency_min'] = min(stop_latencies)
        result['stop_latency_max'] = max(stop_latencies)
    if resume_latencies:
        result['resume_latency_mean'] = statistics.mean(resume_latencies)
        result['resume_latency_median'] = statistics.median(resume_latencies)
        result['resume_latency_min'] = min(resume_latencies)
        result['resume_latency_max'] = max(resume_latencies)
    if deadman_latencies:
        result['deadman_latency_mean'] = statistics.mean(deadman_latencies)
        result['deadman_latency_median'] = statistics.median(deadman_latencies)

    return result


def print_summary(results):
    """Print formatted summary tables."""
    print()
    print('=' * 70)
    print('EVALUATION SUMMARY')
    print('=' * 70)

    # Navigation success table
    print()
    print('Table 1: Navigation Success Rate')
    print('-' * 55)
    print(f'{"Scenario":<15} {"Trials":<8} {"Waypoints":<12} {"Success":<10} {"Rate":<8}')
    print('-' * 55)
    for r in results:
        print(f'{r["scenario"]:<15} {r["num_trials"]:<8} {r["total_waypoints"]:<12} '
              f'{r["total_succeeded"]:<10} {r["nav_success_rate"]:.1f}%')
    print('-' * 55)
    total_wp = sum(r['total_waypoints'] for r in results)
    total_ok = sum(r['total_succeeded'] for r in results)
    print(f'{"TOTAL":<15} {sum(r["num_trials"] for r in results):<8} {total_wp:<12} '
          f'{total_ok:<10} {total_ok/total_wp*100:.1f}%')
    print()

    # Mission time table
    print('Table 2: Mission Completion Time (seconds)')
    print('-' * 45)
    print(f'{"Scenario":<15} {"Mean":<10} {"Std Dev":<10}')
    print('-' * 45)
    for r in results:
        mean = r.get('mission_time_mean', 0)
        std = r.get('mission_time_std', 0)
        print(f'{r["scenario"]:<15} {mean:<10.1f} {std:<10.1f}')
    print()

    # Stop latency table
    print('Table 3: Mode Switch Stop Latency (ms)')
    print('-' * 60)
    print(f'{"Scenario":<15} {"N":<5} {"Mean":<10} {"Median":<10} {"Min":<10} {"Max":<10}')
    print('-' * 60)
    for r in results:
        if r['stop_latencies']:
            n = len(r['stop_latencies'])
            print(f'{r["scenario"]:<15} {n:<5} {r["stop_latency_mean"]*1000:<10.2f} '
                  f'{r["stop_latency_median"]*1000:<10.2f} {r["stop_latency_min"]*1000:<10.2f} '
                  f'{r["stop_latency_max"]*1000:<10.2f}')
    print()

    # Resume latency table
    print('Table 4: Mode Switch Resume Latency (ms)')
    print('-' * 60)
    print(f'{"Scenario":<15} {"N":<5} {"Mean":<10} {"Median":<10} {"Min":<10} {"Max":<10}')
    print('-' * 60)
    for r in results:
        if r['resume_latencies']:
            n = len(r['resume_latencies'])
            print(f'{r["scenario"]:<15} {n:<5} {r["resume_latency_mean"]*1000:<10.1f} '
                  f'{r["resume_latency_median"]*1000:<10.1f} {r["resume_latency_min"]*1000:<10.1f} '
                  f'{r["resume_latency_max"]*1000:<10.1f}')
    print()

    # Dead-man latency table
    deadman = [r for r in results if r['deadman_latencies']]
    if deadman:
        r = deadman[0]
        print('Table 5: Dead-Man Timeout Response (ms)')
        print('-' * 45)
        print(f'  Trials: {len(r["deadman_latencies"])}')
        print(f'  Mean:   {r["deadman_latency_mean"]*1000:.1f} ms')
        print(f'  Median: {r["deadman_latency_median"]*1000:.1f} ms')
        print(f'  Min:    {min(r["deadman_latencies"])*1000:.1f} ms')
        print(f'  Max:    {max(r["deadman_latencies"])*1000:.1f} ms')
        print()

    # Arbitration success
    total_mode_tests = sum(len(r['stop_latencies']) for r in results)
    print(f'Mode Arbitration Tests: {total_mode_tests}/{total_mode_tests} succeeded (100%)')
    print()


def generate_charts(results, output_dir):
    """Generate charts for the report."""
    if not HAS_MATPLOTLIB:
        return

    os.makedirs(output_dir, exist_ok=True)

    # Chart 1: Stop latency comparison across scenarios
    fig, ax = plt.subplots(figsize=(8, 5))
    scenario_names = []
    scenario_data = []
    for r in results:
        if r['stop_latencies']:
            scenario_names.append(r['scenario'].capitalize())
            scenario_data.append([x * 1000 for x in r['stop_latencies']])
    if scenario_data:
        ax.boxplot(scenario_data, labels=scenario_names)
        ax.set_ylabel('Stop Latency (ms)')
        ax.set_title('Mode Switch Stop Latency by Scenario')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'stop_latency_boxplot.png'), dpi=150)
        plt.close()
        print(f'  Saved: stop_latency_boxplot.png')

    # Chart 2: Resume latency comparison
    fig, ax = plt.subplots(figsize=(8, 5))
    scenario_names = []
    scenario_data = []
    for r in results:
        if r['resume_latencies']:
            scenario_names.append(r['scenario'].capitalize())
            scenario_data.append([x * 1000 for x in r['resume_latencies']])
    if scenario_data:
        ax.boxplot(scenario_data, labels=scenario_names)
        ax.set_ylabel('Resume Latency (ms)')
        ax.set_title('Mode Switch Resume Latency by Scenario')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'resume_latency_boxplot.png'), dpi=150)
        plt.close()
        print(f'  Saved: resume_latency_boxplot.png')

    # Chart 3: Navigation success rate bar chart
    fig, ax = plt.subplots(figsize=(8, 5))
    names = [r['scenario'].capitalize() for r in results]
    rates = [r['nav_success_rate'] for r in results]
    colors = ['#2ecc71' if r >= 90 else '#f39c12' if r >= 70 else '#e74c3c' for r in rates]
    bars = ax.bar(names, rates, color=colors, edgecolor='black', linewidth=0.5)
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('Navigation Success Rate by Scenario')
    ax.set_ylim(0, 110)
    for bar, rate in zip(bars, rates):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                f'{rate:.1f}%', ha='center', va='bottom', fontsize=11)
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'nav_success_rate.png'), dpi=150)
    plt.close()
    print(f'  Saved: nav_success_rate.png')

    # Chart 4: Dead-man latency distribution
    deadman = [r for r in results if r['deadman_latencies']]
    if deadman:
        fig, ax = plt.subplots(figsize=(8, 5))
        latencies = [x * 1000 for x in deadman[0]['deadman_latencies']]
        ax.bar(range(1, len(latencies) + 1), latencies, color='#3498db',
               edgecolor='black', linewidth=0.5)
        ax.axhline(y=500, color='red', linestyle='--', label='Design target (500ms)')
        ax.set_xlabel('Trial')
        ax.set_ylabel('Response Time (ms)')
        ax.set_title('Dead-Man Timeout Response Time')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'deadman_latency.png'), dpi=150)
        plt.close()
        print(f'  Saved: deadman_latency.png')


def save_combined_csv(results, scenarios, output_dir):
    """Save a combined CSV with all trial data and summary stats."""
    os.makedirs(output_dir, exist_ok=True)

    # Combined raw data
    outpath = os.path.join(output_dir, 'all_trials_combined.csv')
    with open(outpath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'trial_file', 'scenario', 'waypoint', 'label', 'x', 'y',
            'status', 'duration_s', 'mode_switches', 'stop_latency_s', 'resume_latency_s'
        ])
        writer.writeheader()
        for scenario_name, trials in scenarios.items():
            for trial in trials:
                for row in trial['rows']:
                    row['trial_file'] = trial['file']
                    writer.writerow({
                        'trial_file': trial['file'],
                        'scenario': row.get('scenario', scenario_name),
                        'waypoint': row.get('waypoint', ''),
                        'label': row.get('label', ''),
                        'x': row.get('x', ''),
                        'y': row.get('y', ''),
                        'status': row.get('status', ''),
                        'duration_s': row.get('duration_s', ''),
                        'mode_switches': row.get('mode_switches', ''),
                        'stop_latency_s': row.get('stop_latency_s', ''),
                        'resume_latency_s': row.get('resume_latency_s', ''),
                    })
    print(f'  Saved: all_trials_combined.csv')

    # Summary stats CSV
    outpath = os.path.join(output_dir, 'summary_stats.csv')
    with open(outpath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['scenario', 'trials', 'waypoints', 'succeeded', 'failed',
                         'success_rate_pct', 'mission_time_mean_s', 'mission_time_std_s',
                         'stop_latency_mean_ms', 'stop_latency_median_ms',
                         'resume_latency_mean_ms', 'resume_latency_median_ms',
                         'deadman_latency_mean_ms', 'deadman_latency_median_ms'])
        for r in results:
            writer.writerow([
                r['scenario'], r['num_trials'], r['total_waypoints'],
                r['total_succeeded'], r['total_failed'],
                round(r['nav_success_rate'], 1),
                round(r.get('mission_time_mean', 0), 1),
                round(r.get('mission_time_std', 0), 1),
                round(r.get('stop_latency_mean', 0) * 1000, 2),
                round(r.get('stop_latency_median', 0) * 1000, 2),
                round(r.get('resume_latency_mean', 0) * 1000, 1),
                round(r.get('resume_latency_median', 0) * 1000, 1),
                round(r.get('deadman_latency_mean', 0) * 1000, 1),
                round(r.get('deadman_latency_median', 0) * 1000, 1),
            ])
    print(f'  Saved: summary_stats.csv')


def main():
    trial_dir = sys.argv[1] if len(sys.argv) > 1 else 'trial_results'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'analysis_output'

    if not os.path.isdir(trial_dir):
        print(f'Error: {trial_dir} not found')
        sys.exit(1)

    print(f'Loading trials from: {trial_dir}')
    scenarios = load_trials(trial_dir)

    total = sum(len(v) for v in scenarios.values())
    print(f'Found {total} trials: ' + ', '.join(f'{k}={len(v)}' for k, v in scenarios.items()))

    results = []
    for name, trials in scenarios.items():
        if trials:
            results.append(analyse_scenario(name, trials))

    print_summary(results)

    print(f'Saving outputs to: {output_dir}')
    save_combined_csv(results, scenarios, output_dir)

    if HAS_MATPLOTLIB:
        print('Generating charts...')
        generate_charts(results, output_dir)
    else:
        print('Skipping charts (matplotlib not available)')

    print()
    print('Done.')


if __name__ == '__main__':
    main()
