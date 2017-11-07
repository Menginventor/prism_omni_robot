
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
