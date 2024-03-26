function phi_c_sat = course_roll(chi_c, chi, flag, AP)
  persistent integrator_course
  persistent error_course

  if flag == 0
      integrator_course = 0;
      error_course = 0;
  end
  error = chi_c - chi;
  integrator_course = integrator_course + AP.Ts/2*(error + error_course);
  error_course = error;
  phi_c_sat = sat(AP.course_kp*error + AP.course_ki*integrator_course, AP.e_phi_max, -AP.e_phi_max);
  if AP.course_ki ~= 0
      phi_c_unsat = AP.course_kp*error + AP.course_ki*integrator_course;
      integrator_course = integrator_course + AP.Ts/AP.course_ki*(phi_c_sat - phi_c_unsat);
  end
end

function out = sat(in, up_limit, low_limit)
    if in > up_limit, out = up_limit;
    elseif in < low_limit, out = low_limit;
    else, out = in;
    end
end