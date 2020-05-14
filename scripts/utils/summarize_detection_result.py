
import os
import matplotlib.pyplot as plt
import numpy as np
import json
import pdb
import click

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


@click.command()
@click.option('--save-dir', '-s',
              default='{}/../../data'.format(SCRIPT_DIR))
@click.option('--json-name', '-j', default="2020:05:13_16:30:31.json")
def main(save_dir, json_name):
    json_path = os.path.join(save_dir, json_name)
    with open(json_path) as f:
        result_dat = json.loads(f.read())

    detected_root_points = []
    for frame in result_dat.keys():
        for k in result_dat[frame].keys():
            detected_root_points.append(result_dat[frame][k])
    points_ary = np.asarray(detected_root_points)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    ax.scatter(points_ary[:,1], points_ary[:,0])
    ax.set_title('Detected Root Points')
    ax.set_xlabel('Horisontal-axis[m]')
    ax.set_ylabel('Depth[m]')
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(0.3, 0.7)
    ax.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
