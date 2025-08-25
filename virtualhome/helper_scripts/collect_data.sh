python collect_data_v2.py --scene_ids 4 --clean_surfaces wallshelf kitchentable desk --clean_ids 31 34 132 108 --n_runs_per_scene 16
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_00/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_01/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_02/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_03/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_04/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_05/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_06/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_07/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_08/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_09/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_10/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_11/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_12/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_13/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_14/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene4_15/0
python collect_data_gt.py --datanames scene4_00 scene4_01 scene4_02 scene4_03 scene4_04 scene4_05 scene4_06 scene4_07 scene4_08 scene4_09 scene4_10 scene4_11 scene4_12 scene4_13 scene4_14 scene4_15
python collect_data_v2.py --scene_ids 10 --clean_surfaces wallshelf kitchentable desk --clean_ids 480 136 287 290 289 285 300  301 299 --n_runs_per_scene 16
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_00/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_01/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_02/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_03/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_04/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_05/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_06/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_07/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_08/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_09/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_10/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_11/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_12/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_13/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_14/0
python scripts/amend_missing_files.py --folder /robodata/taijing/benchmarks/virtualhome/unity_output/scene10_15/0
python collect_data_gt.py --datanames scene10_00 scene10_01 scene10_02 scene10_03 scene10_04 scene10_05 scene10_06 scene10_07 scene10_08 scene10_09 scene10_10 scene10_11 scene10_12 scene10_13 scene10_14 scene10_15
python collect_data_v2.py --scene_ids 15 --clean_surfaces wallshelf desk --n_runs_per_scene 16 --port 18080

python collect_data_foods.py --port 18080 --clean_ids 203 208 207 206 210 31 34 132 105 108 109 212 213  --clean_surfaces desk --start_run_id 2 --n_runs_per_scene 14 --seed 43
python collect_data_foods.py --port 8080 --scene_ids 10 --clean_ids 480 136 287 290 289 285 300  301 299 97 101 102 438 439 440 441 442 443 444 445 --clean_surfaces 
python collect_data_foods.py --port 18080 --scene_ids 15 --clean_surfaces desk --clean_ids 163 164 165 166 167 168 170 171 172 173 174 175 178 179 180 181 182 250 251 253 254 255 256 257