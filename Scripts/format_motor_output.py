import argparse
import pandas as pd

'''
Formats motor data output into a CSV format that's usable for MDP
If output_file_path is not specified, output file will be saved in
the same directory as the input file with the string _output append
to the end of the input file name. Assumes that input_file type is
a text (.txt) file.
Python Version Required: >= 3.6.5

pip install pandas

To run:
python3 -m format_motor_output <input_file_path> -o <output_file_path>
'''

parser = argparse.ArgumentParser(description='Format motor data output into a CSV')
parser.add_argument('input_path', metavar='input path', type=str, help='Filepath to input file')
parser.add_argument('-o', const=None, help='Filepath to output file')

def format_data(input_filepath, output_filepath):
    motor_data_file = open(input_filepath, "r")
    motor_data_obj = {}
    active_pwm = None
    line_count = 0

    for line in motor_data_file:
        line_count = line_count + 1
        try:
            tick_value = int(line)
            motor_data_obj[active_pwm].append(tick_value)
        except Exception as e:
            if ("Done" in line):
                break
            pwm_value =  int(line.split(':')[1])
            motor_data_obj[pwm_value] = []
            active_pwm = pwm_value

    print(f'Read in {line_count} number of lines.')

    motor_data_df = pd.DataFrame(data=motor_data_obj)
    motor_data_df = motor_data_df[motor_data_df.columns[::-1]]
    motor_data_df.loc['mean'] = motor_data_df.mean()

    rpm_col = [x for x in range(1, motor_data_df.shape[0])]
    rpm_col.append('Avg')
    motor_data_df.insert(0, 'RPM', rpm_col)

    if output_filepath is None:
        output_filepath = input_filepath.split('.txt')[0] + '_output.csv'

    motor_data_df.to_csv(output_filepath, index=False, encoding='utf-8')
    

if __name__ == '__main__':
    args = parser.parse_args()
    input_filepath = args.input_path
    output_filepath = args.o
    format_data(input_filepath, output_filepath)
