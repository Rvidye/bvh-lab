# Direction D aggregation

run dir: `results/d1_large_sampled_2026_08_31`
coordinates: 28

## Reconciliation

pair rows read: 165409

All frozen reconciliation identities hold.

## Per scene x ray set

| scene | ray_set | seed | res | rays | cand | valid cand | pairs | rays w/ pairs | invalid % | saturated % | AABB FP % | support |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| intel-sponza.obj | primary | A | 128x128 | 16384 | 140031 | 140031 | 16423 | 4571 | 0.00 | 50.59 | 34.86 | yes |
| intel-sponza.obj | shadow_ao | A | 1024x1024 | 292513 | 2387275 | 2387275 | 43359 | 7861 | 0.00 | 42.39 | 93.16 | yes |
| intel-sponza.obj | shadow_ao | B | 1024x1024 | 292513 | 2388416 | 2388416 | 45005 | 8088 | 0.00 | 42.35 | 92.92 | yes |
| intel-sponza.obj | diffuse_1 | A | 128x128 | 4571 | 41748 | 41748 | 1109 | 169 | 0.00 | 42.16 | 89.65 | yes |
| intel-sponza.obj | diffuse_1 | B | 128x128 | 4571 | 40850 | 40850 | 1073 | 168 | 0.00 | 42.96 | 89.41 | yes |
| intel-sponza.obj | incoherent | A | 128x128 | 16384 | 566122 | 566122 | 62149 | 15203 | 0.00 | 54.22 | 46.40 | yes |
| intel-sponza.obj | incoherent | B | 128x128 | 16384 | 556105 | 556105 | 61379 | 15198 | 0.00 | 54.59 | 45.85 | yes |
| hairball.obj | primary | A | 128x128 | 16384 | 408327 | 408327 | 21411 | 2259 | 0.00 | 22.03 | 85.53 | yes |
| hairball.obj | shadow_ao | A | 128x128 | 2259 | 191568 | 191568 | 9336 | 1210 | 0.00 | 20.95 | 84.79 | yes |
| hairball.obj | shadow_ao | B | 128x128 | 2259 | 191344 | 191344 | 9223 | 1196 | 0.00 | 20.84 | 84.98 | yes |
| hairball.obj | diffuse_1 | A | 128x128 | 2259 | 221160 | 221160 | 10755 | 1272 | 0.00 | 20.88 | 85.35 | yes |
| hairball.obj | diffuse_1 | B | 128x128 | 2259 | 223155 | 223155 | 10673 | 1262 | 0.00 | 20.89 | 85.60 | yes |
| hairball.obj | incoherent | A | 128x128 | 16384 | 1180529 | 1180529 | 65474 | 7561 | 0.00 | 24.92 | 82.95 | yes |
| hairball.obj | incoherent | B | 128x128 | 16384 | 1160887 | 1160887 | 64050 | 7382 | 0.00 | 25.11 | 83.05 | yes |
| bistro.obj | primary | A | 128x128 | 16384 | 94834 | 94834 | 7622 | 1239 | 0.00 | 41.05 | 71.83 | yes |
| bistro.obj | shadow_ao | A | 128x128 | 1239 | 61748 | 61748 | 1785 | 338 | 0.00 | 31.52 | 89.69 | yes |
| bistro.obj | shadow_ao | B | 128x128 | 1239 | 59178 | 59178 | 1782 | 327 | 0.00 | 32.72 | 89.45 | yes |
| bistro.obj | diffuse_1 | A | 128x128 | 1239 | 79165 | 79165 | 2914 | 431 | 0.00 | 30.20 | 87.59 | yes |
| bistro.obj | diffuse_1 | B | 128x128 | 1239 | 75128 | 75128 | 2652 | 412 | 0.00 | 31.89 | 87.98 | yes |
| bistro.obj | incoherent | A | 128x128 | 16384 | 516604 | 516604 | 33211 | 5736 | 0.00 | 35.03 | 76.01 | yes |
| bistro.obj | incoherent | B | 128x128 | 16384 | 521492 | 521492 | 33062 | 5709 | 0.00 | 34.72 | 76.36 | yes |
| san-miguel.obj | primary | A | 128x128 | 16384 | 60394 | 60394 | 6677 | 1994 | 0.00 | 63.35 | 45.50 | yes |
| san-miguel.obj | shadow_ao | A | 1024x1024 | 127679 | 1632789 | 1632789 | 27946 | 8306 | 0.00 | 82.86 | 92.86 | yes |
| san-miguel.obj | shadow_ao | B | 1024x1024 | 127679 | 1631250 | 1631250 | 27327 | 8161 | 0.00 | 82.86 | 93.00 | yes |
| san-miguel.obj | diffuse_1 | A | 1024x1024 | 127679 | 1883930 | 1883930 | 63950 | 19120 | 0.00 | 76.69 | 85.36 | yes |
| san-miguel.obj | diffuse_1 | B | 1024x1024 | 127679 | 1886508 | 1886508 | 63230 | 18960 | 0.00 | 76.61 | 85.46 | yes |
| san-miguel.obj | incoherent | A | 128x128 | 16384 | 405317 | 405317 | 42924 | 10731 | 0.00 | 57.20 | 54.57 | yes |
| san-miguel.obj | incoherent | B | 128x128 | 16384 | 390331 | 390331 | 41099 | 10545 | 0.00 | 57.68 | 54.30 | yes |

## Within-parent ranking accuracy (%), ties never counted as correct

| scene | ray_set | seed | directional | dir 95% CI | surf_density | prim_count | box_SA | box_proj | best control | best ctl | delta pp | delta 95% CI |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| intel-sponza.obj | primary | A | 45.55 | [44.78, 46.30] | 49.20 | 27.36 | 42.79 | 43.09 | surface_density | 49.20 | -3.65 | [-4.21, -3.08] |
| intel-sponza.obj | shadow_ao | A | 51.96 | [51.49, 52.44] | 51.91 | 39.18 | 45.27 | 44.50 | surface_density | 51.91 | +0.06 | [-0.40, +0.48] |
| intel-sponza.obj | shadow_ao | B | 52.24 | [51.77, 52.70] | 52.01 | 39.70 | 45.21 | 44.47 | surface_density | 52.01 | +0.23 | [-0.20, +0.65] |
| intel-sponza.obj | diffuse_1 | A | 53.47 | [50.27, 56.38] | 50.77 | 37.78 | 44.45 | 45.99 | surface_density | 50.77 | +2.71 | [+0.17, +5.25] |
| intel-sponza.obj | diffuse_1 | B | 52.00 | [49.14, 54.67] | 52.28 | 37.93 | 41.94 | 41.29 | surface_density | 52.28 | -0.28 | [-2.81, +2.08] |
| intel-sponza.obj | incoherent | A | 52.18 | [51.81, 52.57] | 53.41 | 35.99 | 41.14 | 41.84 | surface_density | 53.41 | -1.23 | [-1.54, -0.91] |
| intel-sponza.obj | incoherent | B | 52.24 | [51.86, 52.63] | 53.45 | 36.02 | 41.14 | 41.60 | surface_density | 53.45 | -1.21 | [-1.53, -0.90] |
| hairball.obj | primary | A | 58.53 | [57.92, 59.18] | 57.71 | 48.61 | 51.40 | 51.62 | surface_density | 57.71 | +0.82 | [+0.51, +1.12] |
| hairball.obj | shadow_ao | A | 58.69 | [57.69, 59.64] | 58.12 | 47.52 | 50.36 | 51.57 | surface_density | 58.12 | +0.57 | [+0.04, +1.08] |
| hairball.obj | shadow_ao | B | 58.50 | [57.58, 59.47] | 58.02 | 47.71 | 49.80 | 51.13 | surface_density | 58.02 | +0.48 | [-0.07, +0.99] |
| hairball.obj | diffuse_1 | A | 58.41 | [57.51, 59.35] | 57.79 | 47.98 | 50.13 | 51.07 | surface_density | 57.79 | +0.62 | [+0.13, +1.11] |
| hairball.obj | diffuse_1 | B | 58.17 | [57.28, 59.12] | 57.32 | 46.66 | 49.87 | 51.08 | surface_density | 57.32 | +0.85 | [+0.35, +1.39] |
| hairball.obj | incoherent | A | 58.40 | [58.05, 58.77] | 57.58 | 48.21 | 50.68 | 51.81 | surface_density | 57.58 | +0.82 | [+0.61, +1.02] |
| hairball.obj | incoherent | B | 58.60 | [58.24, 58.96] | 57.86 | 48.43 | 50.71 | 52.06 | surface_density | 57.86 | +0.74 | [+0.53, +0.94] |
| bistro.obj | primary | A | 57.70 | [56.64, 58.79] | 57.18 | 32.77 | 53.61 | 53.79 | surface_density | 57.18 | +0.52 | [-0.16, +1.22] |
| bistro.obj | shadow_ao | A | 57.70 | [55.46, 59.90] | 58.21 | 38.38 | 50.92 | 52.66 | surface_density | 58.21 | -0.50 | [-1.73, +0.72] |
| bistro.obj | shadow_ao | B | 57.41 | [55.09, 59.61] | 57.35 | 38.27 | 52.75 | 53.59 | surface_density | 57.35 | +0.06 | [-1.25, +1.39] |
| bistro.obj | diffuse_1 | A | 59.13 | [57.25, 60.96] | 58.37 | 40.36 | 50.27 | 51.78 | surface_density | 58.37 | +0.76 | [-0.31, +1.83] |
| bistro.obj | diffuse_1 | B | 58.30 | [56.28, 60.30] | 57.28 | 39.93 | 49.55 | 50.49 | surface_density | 57.28 | +1.02 | [-0.19, +2.27] |
| bistro.obj | incoherent | A | 57.48 | [56.96, 58.00] | 56.36 | 36.39 | 51.25 | 52.07 | surface_density | 56.36 | +1.12 | [+0.79, +1.47] |
| bistro.obj | incoherent | B | 57.49 | [56.98, 58.01] | 56.72 | 36.85 | 50.82 | 51.48 | surface_density | 56.72 | +0.77 | [+0.42, +1.07] |
| san-miguel.obj | primary | A | 52.28 | [50.86, 53.67] | 52.04 | 27.68 | 47.67 | 47.66 | surface_density | 52.04 | +0.24 | [-0.14, +0.62] |
| san-miguel.obj | shadow_ao | A | 51.05 | [50.47, 51.62] | 49.40 | 33.22 | 36.87 | 37.03 | surface_density | 49.40 | +1.65 | [+1.38, +1.92] |
| san-miguel.obj | shadow_ao | B | 50.76 | [50.19, 51.34] | 49.02 | 33.19 | 36.45 | 36.62 | surface_density | 49.02 | +1.73 | [+1.45, +2.03] |
| san-miguel.obj | diffuse_1 | A | 51.86 | [51.41, 52.30] | 52.20 | 31.52 | 42.69 | 44.76 | surface_density | 52.20 | -0.34 | [-0.57, -0.12] |
| san-miguel.obj | diffuse_1 | B | 51.99 | [51.54, 52.46] | 52.20 | 31.30 | 42.72 | 44.73 | surface_density | 52.20 | -0.21 | [-0.43, +0.03] |
| san-miguel.obj | incoherent | A | 53.50 | [52.99, 53.99] | 52.75 | 35.14 | 42.44 | 44.04 | surface_density | 52.75 | +0.75 | [+0.52, +0.98] |
| san-miguel.obj | incoherent | B | 53.71 | [53.19, 54.20] | 53.07 | 34.35 | 42.15 | 43.56 | surface_density | 53.07 | +0.64 | [+0.40, +0.90] |

## Ties per score (count / discordant pairs)

| scene | ray_set | seed | pairs | directional | surface_density | primitive_count | box_surface_ratio | box_projected_ratio |
|---|---|---|---|---|---|---|---|---|
| intel-sponza.obj | primary | A | 16423 | 3797 (23.12%) | 3760 (22.89%) | 7174 (43.68%) | 4705 (28.65%) | 4605 (28.04%) |
| intel-sponza.obj | shadow_ao | A | 43359 | 1545 (3.56%) | 1439 (3.32%) | 11241 (25.93%) | 1634 (3.77%) | 1627 (3.75%) |
| intel-sponza.obj | shadow_ao | B | 45005 | 1531 (3.40%) | 1427 (3.17%) | 11537 (25.63%) | 1646 (3.66%) | 1642 (3.65%) |
| intel-sponza.obj | diffuse_1 | A | 1109 | 50 (4.51%) | 47 (4.24%) | 312 (28.13%) | 53 (4.78%) | 52 (4.69%) |
| intel-sponza.obj | diffuse_1 | B | 1073 | 56 (5.22%) | 56 (5.22%) | 331 (30.85%) | 65 (6.06%) | 64 (5.96%) |
| intel-sponza.obj | incoherent | A | 62149 | 6803 (10.95%) | 6386 (10.28%) | 19667 (31.64%) | 9360 (15.06%) | 9112 (14.66%) |
| intel-sponza.obj | incoherent | B | 61379 | 6804 (11.09%) | 6388 (10.41%) | 19705 (32.10%) | 9481 (15.45%) | 9253 (15.08%) |
| hairball.obj | primary | A | 21411 | 0 (0.00%) | 0 (0.00%) | 3284 (15.34%) | 0 (0.00%) | 0 (0.00%) |
| hairball.obj | shadow_ao | A | 9336 | 0 (0.00%) | 0 (0.00%) | 1486 (15.92%) | 0 (0.00%) | 0 (0.00%) |
| hairball.obj | shadow_ao | B | 9223 | 0 (0.00%) | 0 (0.00%) | 1496 (16.22%) | 0 (0.00%) | 0 (0.00%) |
| hairball.obj | diffuse_1 | A | 10755 | 0 (0.00%) | 0 (0.00%) | 1693 (15.74%) | 0 (0.00%) | 0 (0.00%) |
| hairball.obj | diffuse_1 | B | 10673 | 0 (0.00%) | 0 (0.00%) | 1744 (16.34%) | 0 (0.00%) | 0 (0.00%) |
| hairball.obj | incoherent | A | 65474 | 0 (0.00%) | 0 (0.00%) | 10592 (16.18%) | 1 (0.00%) | 1 (0.00%) |
| hairball.obj | incoherent | B | 64050 | 0 (0.00%) | 0 (0.00%) | 10361 (16.18%) | 2 (0.00%) | 2 (0.00%) |
| bistro.obj | primary | A | 7622 | 257 (3.37%) | 257 (3.37%) | 1756 (23.04%) | 456 (5.98%) | 428 (5.62%) |
| bistro.obj | shadow_ao | A | 1785 | 53 (2.97%) | 53 (2.97%) | 370 (20.73%) | 92 (5.15%) | 89 (4.99%) |
| bistro.obj | shadow_ao | B | 1782 | 50 (2.81%) | 50 (2.81%) | 358 (20.09%) | 85 (4.77%) | 80 (4.49%) |
| bistro.obj | diffuse_1 | A | 2914 | 105 (3.60%) | 105 (3.60%) | 639 (21.93%) | 175 (6.01%) | 168 (5.77%) |
| bistro.obj | diffuse_1 | B | 2652 | 84 (3.17%) | 84 (3.17%) | 540 (20.36%) | 146 (5.51%) | 138 (5.20%) |
| bistro.obj | incoherent | A | 33211 | 1149 (3.46%) | 1149 (3.46%) | 7949 (23.93%) | 2081 (6.27%) | 1931 (5.81%) |
| bistro.obj | incoherent | B | 33062 | 1124 (3.40%) | 1124 (3.40%) | 7881 (23.84%) | 2071 (6.26%) | 1912 (5.78%) |
| san-miguel.obj | primary | A | 6677 | 825 (12.36%) | 823 (12.33%) | 2058 (30.82%) | 979 (14.66%) | 967 (14.48%) |
| san-miguel.obj | shadow_ao | A | 27946 | 3443 (12.32%) | 3421 (12.24%) | 8498 (30.41%) | 3622 (12.96%) | 3606 (12.90%) |
| san-miguel.obj | shadow_ao | B | 27327 | 3397 (12.43%) | 3382 (12.38%) | 8377 (30.65%) | 3596 (13.16%) | 3574 (13.08%) |
| san-miguel.obj | diffuse_1 | A | 63950 | 7302 (11.42%) | 7285 (11.39%) | 21711 (33.95%) | 8667 (13.55%) | 8343 (13.05%) |
| san-miguel.obj | diffuse_1 | B | 63230 | 7198 (11.38%) | 7186 (11.36%) | 21448 (33.92%) | 8560 (13.54%) | 8229 (13.01%) |
| san-miguel.obj | incoherent | A | 42924 | 3482 (8.11%) | 3454 (8.05%) | 11547 (26.90%) | 4470 (10.41%) | 4431 (10.32%) |
| san-miguel.obj | incoherent | B | 41099 | 3435 (8.36%) | 3411 (8.30%) | 11206 (27.27%) | 4397 (10.70%) | 4356 (10.60%) |

## AABB false-positive rate by Q_raw quintile (valid bins only)

| scene | ray_set | seed | Q1 [0.0,0.2) | Q2 [0.2,0.4) | Q3 [0.4,0.6) | Q4 [0.6,0.8) | Q5 [0.8,1.0] | raw_gt_1 | invalid n |
|---|---|---|---|---|---|---|---|---|---|
| intel-sponza.obj | primary | A | 91.14 (n=8571) | 76.61 (n=4973) | 55.30 (n=18272) | 47.36 (n=9204) | 22.41 (n=28166) | 23.17 (n=70845) | 0 |
| intel-sponza.obj | shadow_ao | A | 98.66 (n=415725) | 95.85 (n=169398) | 93.48 (n=225532) | 93.73 (n=252732) | 92.39 (n=311969) | 90.47 (n=1011919) | 0 |
| intel-sponza.obj | shadow_ao | B | 98.57 (n=416188) | 95.64 (n=169660) | 93.21 (n=226516) | 93.53 (n=253186) | 92.16 (n=311477) | 90.16 (n=1011389) | 0 |
| intel-sponza.obj | diffuse_1 | A | 98.05 (n=6777) | 94.47 (n=3074) | 91.01 (n=4347) | 91.24 (n=4486) | 87.61 (n=5463) | 85.46 (n=17601) | 0 |
| intel-sponza.obj | diffuse_1 | B | 97.90 (n=6563) | 94.59 (n=2902) | 90.15 (n=4093) | 89.95 (n=4350) | 88.26 (n=5394) | 85.43 (n=17548) | 0 |
| intel-sponza.obj | incoherent | A | 94.03 (n=18289) | 80.16 (n=20864) | 60.46 (n=68132) | 56.86 (n=52119) | 34.98 (n=99772) | 40.08 (n=306946) | 0 |
| intel-sponza.obj | incoherent | B | 93.57 (n=16837) | 79.54 (n=19981) | 60.15 (n=66972) | 56.08 (n=49897) | 34.46 (n=98865) | 39.86 (n=303553) | 0 |
| hairball.obj | primary | A | 96.28 (n=154349) | 89.25 (n=76486) | 84.31 (n=45988) | 80.09 (n=26626) | 77.73 (n=14928) | 67.44 (n=89950) | 0 |
| hairball.obj | shadow_ao | A | 96.37 (n=69413) | 89.41 (n=35905) | 84.62 (n=23831) | 80.25 (n=14185) | 76.88 (n=8101) | 63.94 (n=40133) | 0 |
| hairball.obj | shadow_ao | B | 96.36 (n=68846) | 89.53 (n=35413) | 84.75 (n=24678) | 80.65 (n=14315) | 78.20 (n=8225) | 64.40 (n=39867) | 0 |
| hairball.obj | diffuse_1 | A | 96.54 (n=80267) | 90.12 (n=39756) | 85.48 (n=27376) | 81.90 (n=17427) | 78.81 (n=10146) | 64.45 (n=46188) | 0 |
| hairball.obj | diffuse_1 | B | 96.52 (n=81270) | 90.13 (n=40004) | 85.82 (n=27641) | 82.05 (n=17519) | 79.21 (n=10105) | 65.27 (n=46616) | 0 |
| hairball.obj | incoherent | A | 96.06 (n=391771) | 88.81 (n=206738) | 83.40 (n=135629) | 79.54 (n=94805) | 76.99 (n=57445) | 63.44 (n=294141) | 0 |
| hairball.obj | incoherent | B | 96.03 (n=381946) | 88.83 (n=203059) | 83.84 (n=134227) | 79.80 (n=93293) | 77.25 (n=56890) | 63.82 (n=291472) | 0 |
| bistro.obj | primary | A | 97.16 (n=26712) | 79.48 (n=7115) | 70.28 (n=8142) | 61.01 (n=6724) | 54.92 (n=7214) | 58.37 (n=38927) | 0 |
| bistro.obj | shadow_ao | A | 99.14 (n=23999) | 93.21 (n=4785) | 88.17 (n=5241) | 86.49 (n=3967) | 82.77 (n=4296) | 79.76 (n=19460) | 0 |
| bistro.obj | shadow_ao | B | 98.96 (n=22038) | 92.56 (n=4650) | 88.74 (n=5124) | 86.50 (n=3874) | 82.74 (n=4130) | 80.09 (n=19362) | 0 |
| bistro.obj | diffuse_1 | A | 98.71 (n=30443) | 91.75 (n=6374) | 85.73 (n=7188) | 84.32 (n=5389) | 78.32 (n=5862) | 75.88 (n=23909) | 0 |
| bistro.obj | diffuse_1 | B | 98.93 (n=27076) | 91.70 (n=6183) | 87.37 (n=7023) | 84.19 (n=5308) | 80.71 (n=5579) | 77.37 (n=23959) | 0 |
| bistro.obj | incoherent | A | 98.04 (n=186195) | 81.75 (n=37374) | 69.84 (n=43364) | 63.31 (n=34625) | 58.56 (n=34062) | 59.36 (n=180984) | 0 |
| bistro.obj | incoherent | B | 98.17 (n=190965) | 81.78 (n=36943) | 70.07 (n=43182) | 63.55 (n=34605) | 58.94 (n=34720) | 59.54 (n=181077) | 0 |
| san-miguel.obj | primary | A | 91.98 (n=4300) | 78.06 (n=2634) | 54.14 (n=6849) | 55.17 (n=2610) | 27.75 (n=5741) | 38.49 (n=38260) | 0 |
| san-miguel.obj | shadow_ao | A | 96.08 (n=18642) | 92.06 (n=25484) | 83.23 (n=85799) | 86.83 (n=52531) | 84.97 (n=97426) | 94.24 (n=1352907) | 0 |
| san-miguel.obj | shadow_ao | B | 95.46 (n=18584) | 91.52 (n=25875) | 83.63 (n=85755) | 86.83 (n=52215) | 85.44 (n=97094) | 94.37 (n=1351727) | 0 |
| san-miguel.obj | diffuse_1 | A | 94.77 (n=36375) | 87.92 (n=43766) | 75.72 (n=141486) | 79.18 (n=75277) | 67.21 (n=142149) | 88.10 (n=1444877) | 0 |
| san-miguel.obj | diffuse_1 | B | 94.95 (n=38232) | 87.41 (n=44358) | 75.99 (n=141146) | 79.17 (n=75191) | 67.37 (n=142369) | 88.19 (n=1445212) | 0 |
| san-miguel.obj | incoherent | A | 93.51 (n=23010) | 80.63 (n=22981) | 61.02 (n=53318) | 58.66 (n=33402) | 39.32 (n=40756) | 48.73 (n=231850) | 0 |
| san-miguel.obj | incoherent | B | 93.16 (n=21876) | 79.22 (n=21480) | 60.63 (n=51370) | 58.73 (n=31808) | 38.66 (n=38659) | 48.76 (n=225138) | 0 |

## Pooled calibration by bin (all coordinates)

| ratio_bin | candidates | AABB FP | FP % | probe_node_steps | probe_prim_steps | probe_box_tests | probe_tri_tests | fp_node_steps | fp_prim_steps | fp_box_tests | fp_tri_tests |
|---|---|---|---|---|---|---|---|---|---|---|---|
| q_00_10 | 1839063 | 1807654 | 98.29 | 1306038 | 2982648 | 2612076 | 2982648 | 1271679 | 2927197 | 2543358 | 2927197 |
| q_10_20 | 946196 | 898120 | 94.92 | 1876086 | 2165047 | 3752172 | 2165047 | 1731297 | 2024660 | 3462594 | 2024660 |
| q_20_30 | 685038 | 626445 | 91.45 | 2403566 | 1966617 | 4807132 | 1966617 | 2100007 | 1727753 | 4200014 | 1727753 |
| q_30_40 | 633177 | 556904 | 87.95 | 2695829 | 1896772 | 5391658 | 1896772 | 2229815 | 1567296 | 4459630 | 1567296 |
| q_40_50 | 1083208 | 880050 | 81.24 | 3008189 | 2256810 | 6016378 | 2256810 | 2362900 | 1739125 | 4725800 | 1739125 |
| q_50_60 | 635013 | 506416 | 79.75 | 3433149 | 1804956 | 6866298 | 1804956 | 2556280 | 1287557 | 5112560 | 1287557 |
| q_60_70 | 668731 | 549264 | 82.14 | 4024751 | 1868438 | 8049502 | 1868438 | 2962476 | 1313487 | 5924952 | 1313487 |
| q_70_80 | 652939 | 530775 | 81.29 | 4408087 | 1870036 | 8816174 | 1870036 | 3214570 | 1293158 | 6429140 | 1293158 |
| q_80_90 | 675234 | 544017 | 80.57 | 4852361 | 1969339 | 9704722 | 1969339 | 3556288 | 1373622 | 7112576 | 1373622 |
| q_90_100 | 1011769 | 694723 | 68.66 | 5760377 | 2481765 | 11520754 | 2481765 | 3987402 | 1536672 | 7974804 | 1536672 |
| raw_gt_1 | 10165817 | 8303831 | 81.68 | 125499374 | 27198318 | 250998748 | 27198318 | 77811804 | 12884900 | 155623608 | 12884900 |
| invalid | 0 | 0 | nan | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

## Pooled accuracy by scene

| scene | rows | candidates | pairs | directional % | best control | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|---|
| bistro.obj | 7 | 1408149 | 83028 | 57.59 | surface_density | 56.74 | +0.85 | 78.31 |
| hairball.obj | 7 | 3576970 | 190922 | 58.49 | surface_density | 57.73 | +0.75 | 83.80 |
| intel-sponza.obj | 7 | 6120547 | 230497 | 51.70 | surface_density | 52.54 | -0.84 | 83.06 |
| san-miguel.obj | 7 | 7890519 | 273153 | 52.25 | surface_density | 51.81 | +0.43 | 85.09 |

## Pooled accuracy by ray_set

| ray_set | rows | candidates | pairs | directional % | best control | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|---|
| diffuse_1 | 8 | 4451644 | 156356 | 53.05 | surface_density | 53.13 | -0.08 | 85.58 |
| incoherent | 8 | 5297387 | 403348 | 55.39 | surface_density | 55.21 | +0.18 | 69.56 |
| primary | 4 | 703586 | 52133 | 53.52 | surface_density | 54.22 | -0.71 | 70.16 |
| shadow_ao | 8 | 8543568 | 165763 | 52.55 | surface_density | 51.85 | +0.70 | 92.58 |

## Pooled accuracy by family

| family | rows | candidates | pairs | directional % | best control | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|---|
| architectural | 21 | 15419215 | 586678 | 52.79 | surface_density | 52.80 | -0.01 | 83.67 |
| organic | 7 | 3576970 | 190922 | 58.49 | surface_density | 57.73 | +0.75 | 83.80 |

## Pooled accuracy by status

| status | rows | candidates | pairs | directional % | best control | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|---|
| diagnostic (development scene) | 28 | 18996185 | 777600 | 54.19 | surface_density | 54.01 | +0.18 | 83.69 |

## Pooled scene-level result

Pooled over 28 supported rows: directional 54.19%, best control `surface_density` 54.01%, delta +0.18 pp.

| scene | rows | directional % | surface_density % | delta pp |
|---|---|---|---|---|
| bistro.obj | 7 | 57.59 | 56.74 | +0.85 |
| hairball.obj | 7 | 58.49 | 57.73 | +0.75 |
| intel-sponza.obj | 7 | 51.70 | 52.54 | -0.84 |
| san-miguel.obj | 7 | 52.25 | 51.81 | +0.43 |

Scene bootstrap (seed 0xD10000D1D10000D1, B=2000, 4 scenes): mean delta +0.30 pp, 95% CI [-0.44, +0.80] pp.

## Gate D1 clause arithmetic

| scene | pooled directional % | min row CI lo % | >=55% and CI>50% |
|---|---|---|---|
| bistro.obj | 57.59 | 55.09 | yes |
| hairball.obj | 58.49 | 57.28 | yes |
| intel-sponza.obj | 51.70 | 44.78 | no |
| san-miguel.obj | 52.25 | 50.19 | no |

clause 1 (>=55% with CI above 50% in >=3 scenes): 2 of 4 scenes qualify
clause 2 (>=+3 pp over best control, scene CI above zero): delta +0.18 pp
substantive scenes available: 0 []; families []
