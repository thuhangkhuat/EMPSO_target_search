% Plot path

function PlotSolution(tours, model)
    MAP_SIZE = model.MAPSIZE;
    x = 1:1:MAP_SIZE; y = 1:1:MAP_SIZE;
    [X,Y] = meshgrid(x,y);

    clf('reset');
    h = pcolor(X,Y,model.Pmap);
    hold on;

    colorList = {'w','r','b','g','m','c','y'};
    nTours = numel(tours);

    for k = 1:nTours
        c = colorList{ mod(k-1, numel(colorList)) + 1 };  

        tourX = tours{k}(:,1) + model.xmax + 0.5;
        tourY = tours{k}(:,2) + model.ymax + 0.5;

        plot(tourX, tourY, [c '-o'], ...
            'MarkerSize', 3, ...
            'MarkerFaceColor', c, ...
            'LineWidth', 1);

 
        plot(tourX(1), tourY(1), [c '-o'], ...
            'MarkerSize', 2, ...
            'MarkerFaceColor', c, ...
            'LineWidth', 1);
    end

    xlabel('x (cell)');
    ylabel('y (cell)');

    set(h, 'linewidth', 0.1);

    cb = colorbar;
    cb.Ruler.Exponent = -3;
    gcaP = get(gca, 'position');
    cbP  = get(cb, 'Position');
    cbP(3) = cbP(3) / 2;              
    set(cb, 'Position', cbP);
    set(gca, 'position', gcaP);
    set(gcf, 'position', [300, 100, 350, 250]);
end