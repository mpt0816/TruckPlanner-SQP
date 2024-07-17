function PAX=prettyAxes(ax) 
% @author:slandarer
% ==================================
% 展示所有可选axes主题 
% prettyAxes().theme()
% ----------------------------------
% prettyAxes(ax).dark()
% prettyAxes().ggray()

if nargin<1
    ax=[];
end

% 从mat文件中读取函数集合
axesTheme=load('axesTheme.mat');
axesTheme=axesTheme.theme;
% 结构体函数构造
for L=1:length(axesTheme.List)
    PAX.(axesTheme.List{L})=@()setAxesTheme(ax,axesTheme,axesTheme.List{L});
end
PAX.theme=@()showAxesTheme(axesTheme);
% =========================================================================
    % 坐标区域修饰基础函数
    function setAxesTheme(tAxes,axesTheme,Name)
        ax=tAxes;
        if isempty(ax)
            ax=gca;
        end
        % 读取函数信息
        sli=0;slii=0;
        tBaseStr=axesTheme.(Name);
        tBaseFunc=axesTheme.([Name,'_F']);
        eval([tBaseStr{:}])

        if ~isempty(tBaseFunc)
            % 设置鼠标移动回调
            set(ax.Parent,'WindowButtonMotionFcn',@bt_move_axes);
        end
        
        % 鼠标移动回调函数
        function bt_move_axes(~,~)
            eval([tBaseFunc{:}])
        end
    end
% -------------------------------------------------------------------------
    % 输出可用风格列表函数
    function showAxesTheme(axesTheme)
        for i=1:length(axesTheme.List)
            fprintf('%s  ',axesTheme.List{i})
        end
        fprintf('\r\n');
    end
end