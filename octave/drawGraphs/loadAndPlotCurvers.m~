A = load("/home/gmanfred/devel/these/projects/SimulationModel2D/error.txt");
%A = A';
x = 10:10:250;

h=figure(1)
plot (x, A(:,1),'color',"magenta",'linewidth',4,x, A(:,2),'color',"red",'linewidth',4,x, A(:,3),'color',"blue",'linewidth',4, x, A(:,4),'color',"green",'linewidth',4, x, A(:,5),'color',"cyan",'linewidth',4);
grid on;

FS = findall(h,'-property','FontSize');
set(FS,'FontSize',14);

xlabel('Matches number','FontSize',20);
ylabel('Rotation error','FontSize',20);
%ylabel('Translation error');
%title('Translation error function of number of matches');

H = 4; W = 6;
set(h,'PaperUnits','inches')
set(h,'PaperOrientation','portrait');
set(h,'PaperSize',[H,W])
set(h,'PaperPosition',[0,0,W,H])

print(h,'-dpng','-color','/home/gmanfred/Documents/myPapers/ICIP2014/src/Figures/test.png')
