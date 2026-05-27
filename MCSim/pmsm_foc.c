/*
 * PMSM FOC Simulation
 * Motor: Surface-mounted PMSM
 * Control: Field-Oriented Control (FOC) with decoupling feed-forward
 * Inverter: Ideal model (no dead-time, no losses)
 *
 * Build:  gcc -O2 pmsm_foc.c -lm -o pmsm_foc
 * Run:    ./pmsm_foc | python3 scope.py
 */

#include <stdio.h>
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define MATHCALC__SATURATE_DIRECT(lim_inf, value, lim_sup)                                          \
 (((value) >= (lim_sup)) ? (lim_sup) : ( ((value) <= (lim_inf)) ? (lim_inf) : (value) ))            \

/* ─── Motor parameters ───────────────────────────────────────────────────── */
#define Rs      26.0       /* Stator resistance [Ω]            */
#define Ld      20.0e-3    /* d-axis inductance [H]            */
#define Lq      20.0e-3    /* q-axis inductance [H]            */
#define LAMBDA  0.07       /* PM flux linkage [Wb]             */
#define P       3          /* Pole pairs                       */
#define J       0.0001      /* Rotor inertia [kg·m²]            */
#define B_FRIC  0.0002      /* Viscous friction [N·m·s/rad]     */

/* ─── DC bus & inverter ──────────────────────────────────────────────────── */
#define VDC     310.0
#define VMAX    (VDC / 2.0)

/* ─── Simulation timing ──────────────────────────────────────────────────── */
#define TS      1e-5             /* Integration step: 100 kHz        */
#define T_CTRL  1.0/8000.0       /* Control update:   10 kHz         */
#define T_SIM   3.0              /* Total sim time [s]               */
#define T_PRINT 2e-4             /* Output decimation                */

/* ─── PI controller ──────────────────────────────────────────────────────── */
typedef struct { double kp, ki, integ, out_max; } PI;

static double pi_update(PI *c, double err, double dt)
{
    c->integ += c->ki * err * dt;
    if (c->integ >  c->out_max)
    {
    	c->integ =  c->out_max;
    }
    if (c->integ < -c->out_max)
    {
    	c->integ = -c->out_max;
   	}
    double out = c->kp * err + c->integ;
    if (out >  c->out_max)
    {
    	out =  c->out_max;
    }
    if (out < -c->out_max)
    {
    	out = -c->out_max;
    }
    return out;
}

/* ─── Motor state ────────────────────────────────────────────────────────── */
typedef struct {
    double id, iq;
    double theta_e, omega_e, omega_m;
    double Te;
} PMSM;

/* Park: alpha-beta → dq */
static void park(double ia, double ib, double th, double *d, double *q)
{
    double c = cos(th), s = sin(th);
    double ialpha = ia;
    double ibeta  = (ia + 2.0 * ib) / sqrt(3.0);
    *d =  c * ialpha + s * ibeta;
    *q = -s * ialpha + c * ibeta;
}

/* Inverse Park: dq → alpha-beta */
static void ipark(double vd, double vq, double th, double *va, double *vb)
{
    double c = cos(th), s = sin(th);
    *va =  c * vd - s * vq;
    *vb =  s * vd + c * vq;
}

/* Ideal SVM: alpha-beta → abc */
static void svm(double va, double vb, double *a_out, double *b_out, double *c_out)
{
	double temp;

	temp =  va;
    *a_out = MATHCALC__SATURATE_DIRECT(-VMAX,temp,VMAX);

    temp = (-va + sqrt(3.0) * vb) / 2.0;
    *b_out = MATHCALC__SATURATE_DIRECT(-VMAX,temp,VMAX);

    temp = (-va - sqrt(3.0) * vb) / 2.0;
    *c_out = MATHCALC__SATURATE_DIRECT(-VMAX,temp,VMAX);
}

/* PMSM Euler step */
static void pmsm_step(PMSM *m, double va, double vb, double TL, double dt)
{
    double vd, vq;
    park(va, vb, m->theta_e, &vd, &vq);

    m->id += (vd - Rs * m->id + m->omega_e * Lq * m->iq) / Ld * dt;
    m->iq += (vq - Rs * m->iq - m->omega_e * (Ld * m->id + LAMBDA)) / Lq * dt;

    m->Te       = 1.5 * P * (LAMBDA + (Ld - Lq) * m->id) * m->iq;
    m->omega_m += (m->Te - TL - B_FRIC * m->omega_m) / J * dt;
    m->omega_e  = m->omega_m * P;
    m->theta_e += m->omega_e * dt;

    /* --- Limit angle to [0:2pi] -------------------*/
    m->theta_e = MATHCALC__SATURATE_DIRECT(0.0,m->theta_e,2.0 * M_PI);
}

/* Speed reference: ramp to 1000 RPM, hold, step to 500 RPM */
static double speed_ref(double t)
{
    double rpm = (t < 1.2) ? t / 1.2 * 3600.0 :
                 (t < 1.7) ? 3600.0 :
                 (t < 1.8) ? 3600.0 - (t-1.7)*8000 : 2800.0;
    return rpm * (2.0 * M_PI / 60.0) * P;
}

/* Load torque */
static double load_torque(double t)
{
	double torque;

	/*--- Create a Hydralic Load ----------------- */
	torque = 0.1 * t*t;
	if(torque > 0.15)
	{
		torque = 0.15;
	}

//	/*--- Create a Linear Load ----------------- */
//	torque = 0.1 * t;
//	if(torque > 0.15)
//	{
//		torque = 0.15;
//	}
//
//	/*--- Create a Constant Load ----------------- */
//	if(t > 0.8)
//	{
//		torque = 0.15;
//	}

	return torque;
}

int main(void)
{
    PMSM m = {0};

    /* PI gains (tuned for Rs=0.5, Ld=4mH, LAMBDA=0.05) */
    PI pi_spd = { .kp=0.03, .ki=10.0,    .integ=0.0, .out_max=1.0   };
    PI pi_id  = { .kp=90.0, .ki=30000.0, .integ=0.0, .out_max=VMAX  };
    PI pi_iq  = { .kp=90.0, .ki=30000.0, .integ=0.0, .out_max=VMAX  };

    double t=0.0, t_ctrl=0.0, t_print=0.0;
    double iq_ref=0.0, id_ref=0.0;
    double vd_cmd=0.0, vq_cmd=0.0;
    double va=0.0, vb=0.0, vc=0.0;

    /* CSV header — all columns available to the scope */
    puts("t,omega_m_rpm,omega_ref_rpm,id,iq,id_ref,iq_ref,"
         "vd,vq,va,Te,TL,theta_e");
    fflush(stdout);

    while (t < T_SIM)
    {
        double TL    = load_torque(t);
        double w_ref = speed_ref(t);

        /* ── FOC control loop ─────────────────────────────────── */
        if (t >= t_ctrl) {
            t_ctrl += T_CTRL;

            /* Outer: speed → iq* */
            iq_ref = pi_update(&pi_spd, w_ref - m.omega_e, T_CTRL);
            id_ref = 0.0;   /* MTPA for surface PMSM */

            /* Inner: id*, iq* → vd, vq  with decoupling FF */
            double vd_ff = -m.omega_e * Lq * m.iq;
            double vq_ff =  m.omega_e * (Ld * m.id + LAMBDA);
            vd_cmd = pi_update(&pi_id, id_ref - m.id, T_CTRL) + vd_ff;
            vq_cmd = pi_update(&pi_iq, iq_ref - m.iq, T_CTRL) + vq_ff;

            /* Modulation: dq → abc */
            double valpha, vbeta;
            ipark(vd_cmd, vq_cmd, m.theta_e, &valpha, &vbeta);
            svm(valpha, vbeta, &va, &vb, &vc);
        }

        /* ── Plant ────────────────────────────────────────────── */
        pmsm_step(&m, va, vb, TL, TS);

        /* ── Output ───────────────────────────────────────────── */
        if (t >= t_print) {
            t_print += T_PRINT;
            printf("%.5f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                   t,
                   m.omega_m * 60.0 / (2.0 * M_PI),
                   w_ref / P * 60.0 / (2.0 * M_PI),
                   m.id, m.iq, id_ref, iq_ref,
                   vd_cmd, vq_cmd, va,
                   m.Te, TL, m.theta_e);
            fflush(stdout);
        }

        t += TS;
    }
    return 0;
}
