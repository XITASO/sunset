import os
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import numpy as np
from typing import Dict


def filter_logs_by_timestamp(
    df: pd.DataFrame, lower_threshold: float, upper_threshold: float
) -> pd.DataFrame:
    """
    Filter the DataFrame based on the timestamp column.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.
    lower_threshold (float): The lower threshold for the timestamp.
    upper_threshold (float): The upper threshold for the timestamp.

    Returns:
    pd.DataFrame: The filtered DataFrame.
    """
    filtered_df = df[
        (df["timestamp"] > lower_threshold) & (df["timestamp"] < upper_threshold)
    ]
    return filtered_df


def calculate_mean_reaction_time(df: pd.DataFrame, filename: str) -> list[float] | None:
    """
    Calculates the mean reaction time from rule trigger to strategy finished successfully.
    """
    # idea staore each triggered rule and wait until we find the resolving strat
    analyzer_logs = df[(df['source'] == 'analyzer')].sort_values("timestamp").values
    current_triggered_rules = {}
    react_times = []
    # if no adaptation given and status is 0 its the first time, add rule, but do check
    for log in analyzer_logs:
        # print(filename)
        # print(current_triggered_rules, log[3], log[4])
        if log[4] is np.nan and log[6] == 0:
            assert(not log[3] in current_triggered_rules)
            # save rule trigger time
            current_triggered_rules[log[3]] = log[0]
        else:
            # we should know that strat already
            assert(log[3] in current_triggered_rules)
            # if we resolved the strat, i.e. status 3 or 5, safe the time diff
            if log[6] == 4:
                pass
            elif log[6] == 3:
                react_times.append(log[0] - current_triggered_rules[log[3]])
                del current_triggered_rules[log[3]]
            elif log[6] == 5:
                react_times.append(log[0] - current_triggered_rules[log[3]])
                del current_triggered_rules[log[3]]

    if len(current_triggered_rules) != 0:
        print("WARNING:, %d, unresolves rules" %len(current_triggered_rules))

    return react_times
    

def calculate_system_down_time(df: pd.DataFrame, filename: str) -> float | None:
    """
    Calcualte the time from when a failure case is produced until the strategy finished successfully.
    """
    segmentation_msg_timestamps = df[df["source"] == "/evaluator_log"].sort_values("timestamp")["timestamp"].values
    # if len(segmentation_msg_timestamps) 
    # .sort_values("timestamp")
    diffs = np.diff(segmentation_msg_timestamps)
    # missing at least one frame
    threshold = 200000000
    diffs_masked = diffs[diffs > threshold]
    down_time = diffs_masked.sum()      
    
    return down_time

def calculate_new_success_metric(df: pd.DataFrame):
    try:
        triggered_strategies = len(df[(df["source"] == "planning") & (df["strategy_status"] == 1)])
        successful_strategies = len(df[(df["source"] == "analyzer") & (df["strategy_status"] == 3)])
        resolved_strategies = len(df[(df["source"] == "analyzer") & (df["strategy_status"] == 5)])
        return (resolved_strategies + successful_strategies) / triggered_strategies
    except:
        return 0


def mean_Iou(df: pd.DataFrame) -> list[float] | None:
    """Calculate the mean IoU from evaluator logs."""
    rows = df[df["source"] == "/evaluator_log"]
    if len(rows) == 0:
        return None
    ious = rows["iou"].astype(float)
    return ious.tolist()

def round_to_significant_figures(num: float, sig_figs: int = 3) -> str:
    """
    Round a number to a specific number of significant figures.

    Parameters:
    num (float): The number to be rounded.
    sig_figs (int): The number of significant figures to retain.

    Returns:
    str: The rounded number as a string.
    """
    if num == 0 or np.isnan(num):
        return str(0)
    else:
        # Determine the number of digits before the decimal point
        num_digits = int(f"{num:e}".split('e')[1])  # Get exponent from scientific notation
        # Compute the shift needed to maintain significant figures
        shift = sig_figs - num_digits - 1
        str_num = str(round(num, shift))
        exp_chars = sig_figs if shift == 0 else sig_figs + 1
        if len(str_num) < exp_chars:
            str_num = str_num + "0"
        return str_num[:exp_chars]

def calculate_unnecessary_redeploys(df: pd.DataFrame):
    try:
        redeploys_necessary = len(df[(df["source"] == "scenario_executor") & (df["gt_failure_name"].str.contains("hard"))])
        redeploys_performed = len(df[(df["source"] == "planning") & (df["strategy_name"].str.contains("redeploy"))])
        return redeploys_performed - redeploys_necessary
    except:
        return 0

def is_valid_log_file(df: pd.DataFrame) -> bool:
    timestamp_col = df["timestamp"].tolist()

    if len(timestamp_col) == 0:
        return False

    if 0 in timestamp_col:
        return False

    for timestamp in timestamp_col:
        stamp = str(timestamp)
        if stamp.startswith("176") and len(stamp) >= 19:
            return False

    evaluator_rows = df[df["source"] == "/evaluator_log"]
    if len(evaluator_rows) < 10:
        return False

    analyzer_rows = df[df["source"] == "analyzer"]
    if len(analyzer_rows) == 0:
        return False

    return True

def main() -> None:
    log_file_folder = "/home/dockuser/ros_ws/log_dump"
    scenarios = ['.']


    overall_result = {}
    for scenario_folder in scenarios:
        scenarion_down_times = []
        scenario_reaction_times = []
        scenario_strategy_success = []
        unnecessary_redeploys = []
        scenario_mIou = []
        for single_run in sorted(
            os.listdir(os.path.join(log_file_folder, scenario_folder))
        ):
            full_path = os.path.join(log_file_folder, scenario_folder, single_run)
            if not 'csv' in full_path:
                continue
            df = pd.read_csv(full_path)
            df.drop(df.tail(4).index,inplace=True) # drop last n rows
            df = df[:df.index[df["source"].str.contains('evaluator')==False][-1] + 10]

            unn_redeploys = calculate_unnecessary_redeploys(df)
            unnecessary_redeploys.append(unn_redeploys)

            if scenario_folder == "settings_deps" and unn_redeploys > 2:
               continue
            log_file_valid = is_valid_log_file(df)

            if not log_file_valid:
                print(f"{full_path} doesn't seem to be a valid logfile")
                continue

            relative_strategy_success = calculate_new_success_metric(df)
            if relative_strategy_success is not None:
                scenario_strategy_success.append(relative_strategy_success)
            reaction_time = calculate_mean_reaction_time(df, full_path)
            if reaction_time is not None:
                scenario_reaction_times.extend(reaction_time)
            down_time = calculate_system_down_time(df, full_path)
            if down_time is not None:
             scenarion_down_times.append(down_time)
            mean_iou = mean_Iou(df)
            if mean_iou is not None:
                scenario_mIou.extend(mean_iou)

        overall_result[scenario_folder] = {}

        overall_result[scenario_folder]["unnecessary_redeploys"] = {
            "mean": np.mean(unnecessary_redeploys),
            "std": np.std(unnecessary_redeploys),
        }

        overall_result[scenario_folder]["strategy_success"] = {
            "mean": np.mean(scenario_strategy_success) if len(scenario_strategy_success) > 0 else np.nan,
            "std": np.std(scenario_strategy_success) if len(scenario_strategy_success) > 0 else np.nan,
        }

        overall_result[scenario_folder]["reaction_time"] = {
           "mean": np.mean(scenario_reaction_times) / 1e9 if len(scenario_reaction_times) > 0 else np.nan,
            "std": np.std(scenario_reaction_times) / 1e9 if len(scenario_reaction_times) > 0 else np.nan,
        }

        overall_result[scenario_folder]["down_time"] = {
            "mean": np.mean(scenarion_down_times) / 1e9 if len(scenarion_down_times) > 0 else np.nan,
            "std": np.std(scenarion_down_times) / 1e9 if len(scenarion_down_times) > 0 else np.nan,
        }

        overall_result[scenario_folder]["mIoU"] = {
            "mean": np.mean(scenario_mIou) if len(scenario_mIou) > 0 else np.nan,
            "std": np.std(scenario_mIou) if len(scenario_mIou) > 0 else np.nan,
        }

    print_latex(result_dict=overall_result)

def print_latex(result_dict: Dict[str, Dict[str, Dict[str, float]]]) -> None:
    """Print a LaTeX table summarizing managing system evaluation.

    Expected structure per scenario key:
    result_dict[scenario] = {
        'adaptations': {'mean': float, 'std': float},
        'reaction_time': {'mean': float, 'std': float},
        'down_time': {'mean': float, 'std': float},
        'mIoU': {'mean': float, 'std': float}
    }
    Scenario key naming is assumed to encode flags via suffixes _deps, _crit, _cost.
    True is rendered as 'x', False as '\\checkmark'.
    """
    def flag(active: bool) -> str:
        return '\\checkmark' if active else 'x'
    
    print("\n\n\n")

    # Print LaTeX table header
    print("\\begin{table*}")
    print("\\begin{tabular}{ccccccc}")
    print("\\toprule")
    print(r"\frac{$Strat_{resolved}$}{$Strat_{exec}$} & Reaction Time (s) & \# $redeploys_{unneccessary}$ & System Downtime (s) & IoU  \\")
    print("\\midrule")

    latex_table = []

    for scenario in sorted(result_dict.keys()):
        data = result_dict[scenario]

        unn_redeploy_mean = data['unnecessary_redeploys']['mean']
        unn_redeploy_std = data['unnecessary_redeploys']['std']

        strat_success_mean = data['strategy_success']['mean']
        strat_success_std = data['strategy_success']['std']

        rt_mean = data['reaction_time']['mean']
        rt_std = data['reaction_time']['std']

        dt_mean = data['down_time']['mean']
        dt_std = data['down_time']['std']

        miou_mean = data['mIoU']['mean']
        miou_std = data['mIoU']['std']

        if np.isnan(unn_redeploy_mean) and np.isnan(unn_redeploy_std):
            unn_redeploys = "N/A"
        else:
            unn_redeploys = f"{round_to_significant_figures(unn_redeploy_mean)} $\\pm$ {round_to_significant_figures(unn_redeploy_std)}"

        if np.isnan(strat_success_mean) and np.isnan(strat_success_std):
            strat_success = "N/A"
        else:
            strat_success = f"{round_to_significant_figures(strat_success_mean )} $\\pm$ {round_to_significant_figures(strat_success_std )}"

        if np.isnan(rt_mean) and np.isnan(rt_std):
            reaction_time = "N/A"
        else:
            reaction_time = f"{round_to_significant_figures(rt_mean)} $\\pm$ {round_to_significant_figures(rt_std)}"

        if np.isnan(dt_mean) and np.isnan(dt_std):
            down_time = "N/A"
        else:
            down_time = f"{round_to_significant_figures(dt_mean)} $\\pm$ {round_to_significant_figures(dt_std)}"

        if np.isnan(miou_mean) and np.isnan(miou_std):
            miou = "N/A"
        else:
            miou = f"{round_to_significant_figures(miou_mean)} $\\pm$ {round_to_significant_figures(miou_std)}"

        latex_table.append(f"{strat_success} & {reaction_time} & {unn_redeploys} & {down_time}& {miou} \\\\")
    # Print table rows and footer
    print("\n".join(latex_table))
    print("\\bottomrule")
    print("\\end{tabular}")
    print("\\begin{table*}")


if __name__ == "__main__":
    main()