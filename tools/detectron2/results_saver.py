#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================


import os
from tomark import Tomark

def save_res(results, out_dir=''):
    """
    Save experimental results as an MD table.

    Args:
        results_dict (dict): a result dictionary.
        out_dir (str): Output directory. If not specified, 
                      'results' dir will be created in the root dir

    """
    
    if not out_dir:
        out_dir = os.path.join(os.getcwd(), 'results')
    os.makedirs(out_dir, exist_ok=True)
    
    for i, res_dict in enumerate(results):
        keys_to_pop = list()
        for key, value in res_dict.items():
            if not key.startswith('AP-'):
                keys_to_pop.append(key)
            else:
                results[i][key] = round(value, 2)
        for key in keys_to_pop:
            results[i].pop(key)
        for old_key in sorted(results[i]):
            results[i][old_key.replace('AP-', '')] = results[i].pop(old_key)
    
    md_table = '## Segmentation results\n\n'
    md_table += Tomark.table(results)

    out_file = open(os.path.join(out_dir, 'results.md'), 'w')
    out_file.write(md_table)
    out_file.close()
