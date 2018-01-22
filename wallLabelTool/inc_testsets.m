global intrinsics baseline gp


%img_path = '/home/schwarze/OIWOB/images';
%intrinsics = [459.458 0 315.445; 0 459.458 234.589; 0 0 1];
%baseline = 0.107486;

% vor-rektifiziertes bild existiert nur für ...e ids!
% in 2011_08_08_Campus_rect für beide bilder

% maschinenhalle
%img_id = '2011_08_08_Campus/stream_b09d01009e410e_5_000190';
%gp = [-0.0597918,-0.981598,-0.181355,1.80472];

% maschinenhalle2
%img_id = '2011_08_08_Campus/stream_b09d01009e410e_5_000374';
%gp = [0.00376316,-0.969454,-0.245244,1.79553];

% maschinenhalle 3
%img_id = '2011_08_08_Campus/stream_b09d01009e410e_5_000441';
%gp = [-0.0374803,-0.982633,-0.181738,1.78586];

% container
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_2_000340';
%gp = [-0.050748,-0.969923,-0.238061,1.81604];

% waldrand
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_000044';
%gp = [-0.0447844,-0.983568,0.174893,2.12839];

%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_000059';
%gp = [-0.155527,-0.984692,-0.0786935,1.83768];

% waldrand 2
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_000009';
%gp = [-0.0769861,-0.986388,-0.145301,1.80565];

% waldrand 3
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_000125';
%gp = [-0.0186545,-0.975366,-0.219802,1.85661];

% kreuzung
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_001292';
%gp = [0.0143833,-0.976012,-0.217242,1.90545];

% kreuzung2
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_001345';
%gp = [0.119023,-0.990649,0.0666938,2.01214];

% kreuzung3
%img_id =  '2011_08_08_Campus/stream_b09d01009e410e_3_001368';
%gp = [-0.0184937,-0.991162,-0.13136,1.939];


% buero
%img_id =  '2011_07_18_Buero/stream_b09d01009e410e_0_000025';
%gp = [0.507298,-0.821926,-0.259011,1.12728];


% -----------------

%%% City dataset

img_path = '/home/schwarze/OIWOB/ext/datasets';
%img_path = '/home/schwarze/OIWOB/images';

intrinsics = [ 402.755 0 317.849 ; 0 402.755 230.576 ; 0 0 1 ];
baseline = 0.129515;

% City1
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_001155';
%gp = [-0.0597918,-0.981598,-0.181355,1.80472];

%hirschstr
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_015126';
%gp = [0.1612   -0.9848    0.0655    1.9838];

%hirschstr 2 
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_015148';
%gp = [0.0723   -0.9939    0.0828    2.0055];


%akademiestr
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_002338';
%gp = [0.0832419,-0.993909,0.0722152,1.7833]

% akademiestr strongly slanted
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_002387';
%gp = [0.2028   -0.9450   -0.2567    1.9585]

%waldstd/hirschstr.
%img_id = '2012_03_21_city/cache/stream_b09d01009e410e_1_010597';
%gp = [0.0825671,-0.987517,0.13414,1.9694]


% ICINCO

% akadamiestr
img_idS{1} = '2012_03_21_city/cache/stream_b09d01009e410e_1_001640';
gpS{1} = [0.0958045 -0.995281  0.0153761 1.99745];

% hirschstr zurück
img_idS{2} = '2012_03_21_city/cache/stream_b09d01009e410e_1_014482';
gpS{2} = [0.0998314 -0.994598 0.0284264 1.863];

% hirschstr zurück2 schwer
img_idS{3} = '2012_03_21_city/cache/stream_b09d01009e410e_1_014904';
gpS{3} = [-0.0504868 -0.997749 0.0441446 2.12105];

% hirschstr zurück3
img_idS{4} = '2012_03_21_city/cache/stream_b09d01009e410e_1_015457';
gpS{4} = [0.122363 -0.987947 0.0948086 1.9347];

% rechts um die ecke
img_idS{5} = '2012_03_21_city/cache/stream_b09d01009e410e_1_015960';
gpS{5} = [0.15454 -0.987977 0.00426325 2.05438]; % poasst nicht

% selbe str weiter
img_idS{6} = '2012_03_21_city/cache/stream_b09d01009e410e_1_016008';
gpS{6} = [0.023023 -0.999154 0.0340912 1.85952];

% noch weiter
img_idS{7} = '2012_03_21_city/cache/stream_b09d01009e410e_1_016119';
gpS{7} = [0.1706 -0.984296 0.0453478 1.98352];

% hecke
img_idS{8} = '2012_03_21_city/cache/stream_b09d01009e410e_1_019435';
gpS{8} = [0.0689492 -0.993803 0.0871842 1.88125];

% spiegel
img_idS{9} = '2012_03_21_city/cache/stream_b09d01009e410e_1_019912';
gpS{9} = [0.083344 -0.995436 0.0464931 1.95049];

% container im weg
img_idS{10} = '2012_03_21_city/cache/stream_b09d01009e410e_1_020576';
gpS{10} = [-0.032025,-0.993245,0.111531,1.96193];

% kaiserstr. völlig unrealistisch
img_idS{11} = '2012_03_21_city/cache/stream_b09d01009e410e_1_028040';
gpS{11} = [-0.0022475 -0.998305 -0.058151 1.83312]; % die is kacke!

%campus
img_idS{12} = '2012_03_21_city/cache/stream_b09d01009e410e_1_033573';
gpS{12} = [0.0464273 -0.998675 -0.0222143 1.92737];


%hirschstr 2 
img_idS{13} = '2012_03_21_city/cache/stream_b09d01009e410e_1_015126';
gpS{13} = [0.1612   -0.9848    0.0655    1.9838];
