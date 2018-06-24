use hal::hal::digital::OutputPin;

pub struct EightSegment<'a> {
    pub seg_a: &'a mut OutputPin,
    pub seg_b: &'a mut OutputPin,
    pub seg_c: &'a mut OutputPin,
    pub seg_d: &'a mut OutputPin,
    pub seg_e: &'a mut OutputPin,
    pub seg_f: &'a mut OutputPin,
    pub seg_g: &'a mut OutputPin,
    pub seg_p: &'a mut OutputPin,
}

impl<'a> EightSegment<'a> {
    pub fn display(&mut self, count: u8, dp: bool) {
        let (seg_a_on, seg_f_on, seg_b_on, seg_g_on, seg_e_on, seg_c_on, seg_d_on) = match count {
            0 => (true, true, true, false, true, true, true),
            1 => (false, false, true, false, false, true, false),
            2 => (true, false, true, true, true, false, true),
            3 => (true, false,  true, true, false, true, true),
            4 => (false, true, true, true, false, true, false),
            5 => (true, true, false, true, false, true, true),
            6 => (true, true, false, true, true, true, true),
            7 => (true, false, true, false, false, true, false),
            8 => (true, true, true, true, true, true, true),
            9 => (true, true, true, true, false, true, false),
            _ => (true, true, true, true, true, true, true),
        };

        if !seg_a_on {self.seg_a.set_high()} else {self.seg_a.set_low()};
        if !seg_b_on {self.seg_b.set_high()} else {self.seg_b.set_low()};
        if !seg_c_on {self.seg_c.set_high()} else {self.seg_c.set_low()};
        if !seg_d_on {self.seg_d.set_high()} else {self.seg_d.set_low()};
        if !seg_e_on {self.seg_e.set_high()} else {self.seg_e.set_low()};
        if !seg_f_on {self.seg_f.set_high()} else {self.seg_f.set_low()};
        if !seg_g_on {self.seg_g.set_high()} else {self.seg_g.set_low()};
        if !dp {self.seg_p.set_high()} else {self.seg_p.set_low()};
    }
}
