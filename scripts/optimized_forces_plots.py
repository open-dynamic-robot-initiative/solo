#!/usr/bin/env python3
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt


def plot_graphs(movement_type="circular"):

    if movement_type == "circular":
        file = open("robot_data_dump_circular_movement.pkl", "rb")
    if movement_type == "up":
        file = open("robot_data_dump_up_1_sec.pkl", "rb")

    data = pickle.load(file)
    file.close()

    N = data["observed_wrench"].shape[0]

    # Plotting Contact Forces
    fig1, ax1 = plt.subplots(9, 1)
    fig1.suptitle('Contact Forces')
    for i in range(3):
        # Optimised Contact Forces (Desired)
        ax1[i].plot(1.e-3 * np.arange(N), data["optimised_forces"][:, i], '--')
        ax1[i + 3].plot(1.e-3 * np.arange(N),
                        data["optimised_forces"][:, i + 3], '--')
        ax1[i + 6].plot(1.e-3 * np.arange(N),
                        data["optimised_forces"][:, i + 6], '--')

        # Observed Contact Forces
        ax1[i].plot(1.e-3 * np.arange(N), data["observed_forces"][:, i])
        ax1[i + 3].plot(1.e-3 * np.arange(N),
                        data["observed_forces"][:, i + 3])
        ax1[i + 6].plot(1.e-3 * np.arange(N),
                        data["observed_forces"][:, i + 6])

    y_axis_label = [
        "F1x",
        "F1y",
        "F1z",
        "F2x",
        "F2y",
        "F2z",
        "F3x",
        "F3y",
        "F3z"]
    for idx, ax in enumerate(ax1.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Plotting Contact Points
    fig2, ax2 = plt.subplots(9, 1)
    fig2.suptitle('Contact Points')
    for i in range(3):
        # Optimised Contact Forces (Desired)
        ax2[i].plot(1.e-3 * np.arange(N),
                    data["desired_contact_points"][:, i], '--')
        ax2[i + 3].plot(1.e-3 * np.arange(N),
                        data["desired_contact_points"][:, i + 3], '--')
        ax2[i + 6].plot(1.e-3 * np.arange(N),
                        data["desired_contact_points"][:, i + 6], '--')

        # Observed Contact Forces
        ax2[i].plot(1.e-3 * np.arange(N),
                    data["observed_contact_points"][:, i])
        ax2[i + 3].plot(1.e-3 * np.arange(N),
                        data["observed_contact_points"][:, i + 3])
        ax2[i + 6].plot(1.e-3 * np.arange(N),
                        data["observed_contact_points"][:, i + 6])

    y_axis_label = [
        "P1x",
        "P1y",
        "P1z",
        "P2x",
        "P2y",
        "P2z",
        "P3x",
        "P3y",
        "P3z"]
    for idx, ax in enumerate(ax2.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Plotting Contact Velocities
    fig3, ax3 = plt.subplots(9, 1)
    fig3.suptitle('Contact Velocities')
    for i in range(3):
        # Optimised Contact Forces (Desired)
        ax3[i].plot(1.e-3 * np.arange(N),
                    data["desired_contact_velocities"][:, i], '--')
        ax3[i + 3].plot(1.e-3 * np.arange(N),
                        data["desired_contact_velocities"][:, i + 3], '--')
        ax3[i + 6].plot(1.e-3 * np.arange(N),
                        data["desired_contact_velocities"][:, i + 6], '--')

        # Observed Contact Forces
        ax3[i].plot(1.e-3 * np.arange(N),
                    data["observed_contact_velocities"][:, i])
        ax3[i + 3].plot(1.e-3 * np.arange(N),
                        data["observed_contact_velocities"][:, i + 3])
        ax3[i + 6].plot(1.e-3 * np.arange(N),
                        data["observed_contact_velocities"][:, i + 6])

    y_axis_label = [
        "V1x",
        "V1y",
        "V1z",
        "V2x",
        "V2y",
        "V2z",
        "V3x",
        "V3y",
        "V3z"]
    for idx, ax in enumerate(ax3.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Plotting Block's Wrench
    fig4, ax4 = plt.subplots(6, 1)
    fig4.suptitle('Wrench')
    for i in range(3):
        # Block COM Forces
        ax4[i].plot(1.e-3 * np.arange(N), data["desired_wrench"][:, i], '--')
        ax4[i].plot(1.e-3 * np.arange(N), data["observed_wrench"][:, i])

        # Block COM Moment
        ax4[i + 3].plot(1.e-3 * np.arange(N),
                        data["desired_wrench"][:, i + 3], '--')
        ax4[i + 3].plot(1.e-3 * np.arange(N),
                        data["observed_wrench"][:, i + 3])

    y_axis_label = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
    for idx, ax in enumerate(ax4.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Plotting Block COM position
    fig5, ax5 = plt.subplots(7, 1)
    fig5.suptitle('Block Pose')
    for i in range(4):
        if i < 3:
            # Block Position
            ax5[i].plot(1.e-3 * np.arange(N),
                        data["desired_block_position"][:, i], '--')
            ax5[i].plot(1.e-3 * np.arange(N),
                        data["observed_block_position"][:, i])

        # Block Orientation
        ax5[i + 3].plot(1.e-3 * np.arange(N),
                        data["desired_block_position"][:, i + 3], '--')
        ax5[i + 3].plot(1.e-3 * np.arange(N),
                        data["observed_block_position"][:, i + 3])

    y_axis_label = ["x", "y", "z", "i", "j", "k", "w"]
    for idx, ax in enumerate(ax5.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Differences in State Variables
    fig6, ax6 = plt.subplots(12, 1)
    fig6.suptitle('Differences')
    for i in range(3):
        # Block Position
        ax6[i].plot(1.e-3 * np.arange(N), data["differences"][:, i], '--')

        # Block Orientation
        ax6[i + 3].plot(1.e-3 * np.arange(N), data["differences"][:, i + 3])

        # Velocity
        ax6[i + 6].plot(1.e-3 * np.arange(N),
                        data["differences"][:, i + 6], '--')

        # Angular Velocity
        ax6[i + 9].plot(1.e-3 * np.arange(N),
                        data["differences"][:, i + 9], 'r-')

    y_axis_label = [
        "ΔPx",
        "ΔPy",
        "ΔPz",
        "ΔOx",
        "ΔOy",
        "ΔOz",
        "ΔVx",
        "ΔVy",
        "ΔVz",
        "ΔWx",
        "ΔWy",
        "ΔWz"]
    for idx, ax in enumerate(ax6.flat):
        ax.set(ylabel=y_axis_label[idx])

    # Showing all of them
    plt.show()


if __name__ == '__main__':
    plot_graphs("circular")
    plot_graphs("up")
