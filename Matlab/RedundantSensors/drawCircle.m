function h = drawCircle(center, radius, varargin)
% h = drawCircle(center, radius, varargin)
% DRAWCIRCLE Draw a circle using rectangle with curvature [1 1].
%
%   h = drawCircle(center, radius) draws a circle centered at
%   center = [x, y] with the given radius. Returns the rectangle handle h.
%
%   h = drawCircle(center, radius, 'EdgeColor', ec, 'FaceColor', fc, ...
%                  'FaceAlpha', a, 'LineWidth', lw)
%   Optional name/value pairs (defaults shown):
%       'EdgeColor'  : [0 0 0]       (black)
%       'FaceColor'  : [1 0 0]       (red)
%       'FaceAlpha'  : 0.25
%       'LineWidth'  : 1.0
%
% Example:
%   figure; hold on; axis equal;
%   drawCircle([1,2], 0.5, 'FaceColor',[0 0.6 0.9], 'FaceAlpha', 0.4, 'EdgeColor','k');
%
% Written by chatGPT on Dec 8 2025

% Input checks
narginchk(2, Inf);
if numel(center) ~= 2
    error('center must be a 2-element vector [x y].');
end
if ~isscalar(radius) || radius <= 0
    error('radius must be a positive scalar.');
end

% Default options
opts.EdgeColor = unifrnd(0, 1, 1, 3);
opts.FaceColor = opts.EdgeColor;
opts.FaceAlpha = 0.25;
opts.LineWidth = 1.0;

% Parse name/value pairs
if ~isempty(varargin)
    if mod(numel(varargin),2)~=0
        error('Optional inputs must be name/value pairs.');
    end
    for k = 1:2:numel(varargin)
        name = varargin{k};
        val  = varargin{k+1};
        switch lower(name)
            case 'edgecolor'
                opts.EdgeColor = val;
            case 'facecolor'
                opts.FaceColor = val;
            case 'facealpha'
                opts.FaceAlpha = val;
            case 'linewidth'
                opts.LineWidth = val;
            case 'name'
                opts.Name = val;
            otherwise
                error('Unknown option "%s".', name);
        end
    end
end

% Make sure axes exist and maintain hold state
ax = gca;
holdState = ishold(ax);
hold(ax, 'on');

% Rectangle position: [x_left, y_bottom, width, height]
pos = [center(1) - radius, center(2) - radius, 2*radius, 2*radius];

% Draw circle (rectangle with full curvature)
h = rectangle('Position', pos, ...
              'Curvature', [1 1], ...
              'EdgeColor', opts.EdgeColor, ...
              'FaceColor', opts.FaceColor, ...
              'FaceAlpha', opts.FaceAlpha, ...
              'LineWidth', opts.LineWidth);

% Keep axis aspect equal so circle looks circular
axis(ax, 'equal');

fake = line(NaN, NaN, LineWidth=opts.LineWidth, Color=opts.FaceColor);
set(fake, 'DisplayName', opts.Name);

% Restore hold state if needed
if ~holdState
    hold(ax, 'off');
end
end
