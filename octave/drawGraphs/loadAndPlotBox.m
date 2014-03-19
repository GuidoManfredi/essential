pkg load statistics

%A = load("/home/gmanfred/devel/these/projects/SimulationModel2D/estimate_rotation_noise_vs_precision.txt");
%A = load("/home/gmanfred/devel/these/projects/SimulationModel2D/estimate_translation_noise_vs_precision.txt");
A = load("/home/gmanfred/devel/these/projects/SimulationModel2D/refine_rotation_noise_vs_precision.txt");
A = A * 100;
A = A'(:, 2:end);

set (0, "defaultlinelinewidth", 2);
set (0, "defaultaxesfontsize", 20);


h=figure(1);
boxplot (A');
grid on;

xlabel("Noise", 'FontSize', 20);
ylabel("Rotation error", 'FontSize', 20);

H = 3.5; W = 4;
set(h,'PaperUnits','inches')
set(h,'PaperOrientation','portrait');
set(h,'PaperSize',[H,W])
set(h,'PaperPosition',[0,0,W,H])

%print(h,'-dpng','-color','/home/gmanfred/Documents/myPapers/ICIP2014/src/Figures/rotation_error_noise2.png')
%print(h,'-dpng','-color','/home/gmanfred/Documents/myPapers/ICIP2014/src/Figures/translation_error_noise2.png')
