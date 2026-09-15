"""Shared publication style for all manuscript and supplementary figures.

Colors are editorial choices, not an ICRA-mandated palette. Typography follows
the manuscript's Times-compatible text and mathematical notation.
"""
import matplotlib.pyplot as plt

BLUE, ORANGE, AQUA = '#1769AA', '#B85A18', '#087F70'
INK, INK2, GRID, SURF = '#18324A', '#526273', '#DDE3E9', '#FFFFFF'
FONT = 'Tinos'


def apply_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': [FONT, 'Tinos', 'Times New Roman', 'Nimbus Roman',
                       'Liberation Serif', 'DejaVu Serif'],
        'mathtext.fontset': 'stix',
        'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8,
        'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 7,
        'axes.edgecolor': INK2, 'axes.linewidth': .7,
        'text.color': INK, 'axes.labelcolor': INK,
        'xtick.color': INK2, 'ytick.color': INK2,
        'grid.color': GRID, 'grid.linewidth': .5,
        'lines.linewidth': 1.1, 'legend.frameon': False,
        'axes.prop_cycle': plt.cycler(color=[BLUE, ORANGE, AQUA]),
        'figure.facecolor': SURF, 'axes.facecolor': SURF,
        'pdf.fonttype': 42, 'ps.fonttype': 42,
        'figure.dpi': 150, 'savefig.dpi': 240,
        'savefig.bbox': 'tight', 'savefig.pad_inches': .03,
    })
