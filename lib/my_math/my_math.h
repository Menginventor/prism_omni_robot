
float constrain(float val,float upper_lim,float under_lim){
        if (val > upper_lim) return upper_lim;
        else if (val < under_lim) return under_lim;
        else return val;

}

mat euler(mat& crr_state,mat (*state_dot)(mat&),float h){
//  crr_state = crr_state+state_dot(crr_state)*float(1.0/100.0);
        mat next_state (crr_state.row,crr_state.col);
        next_state = crr_state+state_dot(crr_state)*h;
        return next_state;
}

mat RK4(mat& crr_state,mat (*state_dot)(mat&),float h){
//  crr_state = crr_state+state_dot(crr_state)*float(1.0/100.0);

        mat K1 (crr_state.row,crr_state.col);
        mat K2 (crr_state.row,crr_state.col);
        mat K3 (crr_state.row,crr_state.col);
        mat K4 (crr_state.row,crr_state.col);

        K1 = state_dot (crr_state);
        mat K2_arg = crr_state+(K1*(0.5*h));
        K2 = state_dot (K2_arg);
        mat K3_arg = crr_state+(K2*0.5*h);
        K3 = state_dot (K3_arg);
        mat K4_arg = crr_state+(K3*h);
        K4 = state_dot (K4_arg);

        mat next_state (crr_state.row,crr_state.col);
        next_state = crr_state+(K1+(K2*2)+(K3*2)+K4)*(h/6.0);

        return next_state;
}
