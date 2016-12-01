function result = check_solution(spline_xyt,start_point,target_point,new_solution_cost, old_solution_cost, figure_to_draw, figure_to_draw_result)

%check the solution

if(old_solution_cost > 0)
    if((new_solution_cost > old_solution_cost) || (new_solution_cost < 0))
        plot_final_spline(spline_xyt,start_point,target_point,figure_to_draw_result,1);
        input('\n The solution is found. \nPress ENTER to exit\n');
        result = 1;
        return;
    end
end

result = -1;