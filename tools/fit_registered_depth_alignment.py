#!/usr/bin/env python3
"""Fit centre-scale and residual-offset calibration from pixel matches."""

import argparse
import json
import math


def _fit_axis(matches, axis, centre):
    source = [float(match['depth'][axis]) - centre for match in matches]
    target = [float(match['rgb'][axis]) - centre for match in matches]
    source_mean = sum(source) / len(source)
    target_mean = sum(target) / len(target)
    variance = sum((value - source_mean) ** 2 for value in source)
    if variance < 1e-9:
        raise ValueError('matches must span distinct positions on both axes')
    scale = sum((x - source_mean) * (y - target_mean)
                for x, y in zip(source, target)) / variance
    offset = target_mean - scale * source_mean
    return scale, offset


def fit_alignment(matches, principal_point):
    """Fit rgb = centre + scale * (registered - centre) + offset."""
    if len(matches) < 4:
        raise ValueError('at least four pixel matches are required')
    centre_x, centre_y = map(float, principal_point)
    scale_x, offset_x = _fit_axis(matches, 0, centre_x)
    scale_y, offset_y = _fit_axis(matches, 1, centre_y)
    squared_error = 0.0
    for match in matches:
        predicted_x = centre_x + scale_x * (
            float(match['depth'][0]) - centre_x) + offset_x
        predicted_y = centre_y + scale_y * (
            float(match['depth'][1]) - centre_y) + offset_y
        squared_error += (predicted_x - float(match['rgb'][0])) ** 2
        squared_error += (predicted_y - float(match['rgb'][1])) ** 2
    return {
        'pixel_scale_x': scale_x,
        'pixel_scale_y': scale_y,
        'pixel_offset_x': offset_x,
        'pixel_offset_y': offset_y,
        'rmse_pixels': math.sqrt(squared_error / (2 * len(matches))),
        'match_count': len(matches),
    }


def main():
    parser = argparse.ArgumentParser(
        description='Fit registered-depth centre scale from JSON pixel matches')
    parser.add_argument('input', help='JSON file containing principal_point and matches')
    args = parser.parse_args()
    with open(args.input, 'r', encoding='utf-8') as stream:
        data = json.load(stream)
    result = fit_alignment(data['matches'], data['principal_point'])
    if not (0.8 <= result['pixel_scale_x'] <= 1.2 and
            0.8 <= result['pixel_scale_y'] <= 1.2):
        raise ValueError('fitted pixel scale is outside node bounds [0.8, 1.2]')
    if abs(result['pixel_offset_x']) > 80 or abs(result['pixel_offset_y']) > 80:
        raise ValueError('fitted residual offset is outside node bounds')
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
