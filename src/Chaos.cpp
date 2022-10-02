#include "plugin.hpp"
#include "osdialog.h"
#include <vector>

#define VOLT_MAX 10.f
#define VOLT_FIVE 5.f
#define POLY_CHANNELS 16
#define PI 3.14159265359
#define INV_SQRT2 0.7071067812

typedef unsigned int uint;
typedef unsigned char uchar;

struct ChaosModule : Module {
	uint frameIndex = 0;
	uint channelCount = 1;
	
	dsp::SchmittTrigger kickTrigger;

	enum ParamIds {
		GRAVITY_PARAM,
		LENGTH_RATIO_PARAM,
		DAMPING_PARAM,
		KICK_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		GRAVITY_IN,
		RATIO_IN,
		DAMPING_IN,
		KICK_TRIG_IN,
		NUM_INPUTS
	};
	enum OutputIds {
		POLY_CHAOS_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		NUM_LIGHTS
	};

	enum IntegrationMode {
		RK4,
		Euler
	};

	enum KickMode {
		KeepVelocity,
		ClearVelocity
	};

	struct Pendulum {
		float theta = 0.0f;
		float length = 0.5f;
		float vel = 0.0f;
		float acc = 0.0f;
		float mass = 10.0f;
		float x, y = 0.f;
	};

	Pendulum p0;
	Pendulum p1;
	int frame = 0;
	IntegrationMode integrationMode;
	KickMode kickMode;

	void Derivative(const Pendulum& p0, const Pendulum& p1, float& out_dxdt0, float& out_dxdt1) {
		const float g = 9.81f;
		const float deltaTheta = p0.theta - p1.theta;
		const float mass_sum = p0.mass + p1.mass;
		const float shared_denominator = (2.f * mass_sum - p1.mass * std::cos(2.f * p0.theta - 2.f * p1.theta));
		const float p0_numerator = 
			-g * (2.f * mass_sum) * std::sin(p0.theta) 
			- p1.mass * g * std::sin(p0.theta - 2.f * p1.theta) 
			- 2.f * std::sin(deltaTheta) * p1.mass 
			* ((p1.vel * p1.vel) * p1.length + (p0.vel * p0.vel) * p0.length * std::cos(deltaTheta));
		const float p0_denominator = p0.length * shared_denominator;
		out_dxdt0 = p0_numerator / p0_denominator;

		const float p1_numerator = 
			2.f * std::sin(deltaTheta) * ((p0.vel * p0.vel) * p0.length * mass_sum 
			+ g * mass_sum * std::cos(p0.theta) 
			+ (p1.vel * p1.vel) * p1.length * p1.mass * std::cos(deltaTheta));
		const float p1_denominator = p1.length * shared_denominator;
		out_dxdt1 = p1_numerator / p1_denominator;
	}

	ChaosModule() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(GRAVITY_PARAM, 0.01f, 6.f, 1.0f, "Timewarp", "x");
		configParam(LENGTH_RATIO_PARAM, 0.1f, 1.0f - 0.1f, 0.5f, "Ratio", "");
		configParam(DAMPING_PARAM, 0.0f, 1.f, 0.0f, "Dampen", "");
		configParam(KICK_PARAM, 0.f, 1.f, 0.f);
		configInput(GRAVITY_IN, "Timewarp CV");
		configInput(RATIO_IN, "Ratio CV");
		configInput(DAMPING_IN, "Damping CV");
		configInput(KICK_TRIG_IN, "Kick trigger");
		configOutput(POLY_CHAOS_OUTPUT, "Poly chaos");
		integrationMode = IntegrationMode::RK4;
		kickMode = KickMode::ClearVelocity;
	}

	void onReset() override {
		Module::onReset();
		p0 = Pendulum{};
		p1 = Pendulum{};
		integrationMode = IntegrationMode::RK4;
		kickMode = KickMode::ClearVelocity;
	}

	void onRandomize() override {
		Module::onRandomize();
		p0.theta = random::uniform() * PI * 2.f;
		p1.theta = random::uniform() * PI * 2.f;
		p0.vel = (random::uniform() - 0.5f) * 5.f;
		p1.vel = (random::uniform() - 0.5f) * 5.f;
	}

	json_t *dataToJson() override {
		json_t *obj = json_object();
		json_object_set_new(obj, "mode", json_integer(integrationMode));
		json_object_set_new(obj, "kick_mode", json_integer(kickMode));
		json_object_set_new(obj, "p0_theta", json_real(p0.theta));
		json_object_set_new(obj, "p0_vel", json_real(p0.vel));
		json_object_set_new(obj, "p1_theta", json_real(p1.theta));
		json_object_set_new(obj, "p1_vel", json_real(p1.vel));
		return obj;
	}

	void dataFromJson(json_t *rootJ) override {
		json_t* modeJ = json_object_get(rootJ, "mode");
		if (modeJ) integrationMode = (IntegrationMode)json_integer_value(modeJ);

		json_t* kickmodeJ = json_object_get(rootJ, "kick_mode");
		if (kickmodeJ) kickMode = (KickMode)json_integer_value(kickmodeJ);

		json_t* p0thetaJ = json_object_get(rootJ, "p0_theta");
		if (p0thetaJ) p0.theta = (float)json_real_value(p0thetaJ);

		json_t* p0velJ = json_object_get(rootJ, "p0_vel");
		if (p0velJ) p0.vel = (float)json_real_value(p0velJ);

		json_t* p1thetaJ = json_object_get(rootJ, "p1_theta");
		if (p1thetaJ) p1.theta = (float)json_real_value(p1thetaJ);

		json_t* p1velJ = json_object_get(rootJ, "p1_vel");
		if (p1velJ) p1.vel = (float)json_real_value(p1velJ);
	}

	float pixelToVoltage(uchar pixel) {
		return ((float)pixel / 255) * VOLT_MAX;
	}

	void kickPendulums() {
		p0.theta = (PI*0.5f) + (random::uniform() * PI);
		p1.theta = 0.0f + (random::uniform() * PI * 2);
		if (kickMode == KickMode::ClearVelocity) {
			p0.vel = 0;
			p1.vel = 0;
			p0.acc = 0;
			p1.acc = 0;
		}
	}

	void process(const ProcessArgs& args) override {
        // Audio signals are typically +/-5V
        // https://vcvrack.com/manual/VoltageStandards.html
		frame++;

		if (frame % 4 == 0) {
			float length_ratio = params[LENGTH_RATIO_PARAM].getValue() + (inputs[RATIO_IN].getVoltageSum() / VOLT_MAX);
			clamp(length_ratio, 0.1f , 0.9f);
			float dt = args.sampleTime * (params[GRAVITY_PARAM].getValue() + (inputs[GRAVITY_IN].getVoltageSum() / VOLT_MAX));
			clamp(dt, 0.f, 6.f);
			float damping_in = (params[DAMPING_PARAM].getValue() + inputs[DAMPING_IN].getVoltageSum());
			float damping = (damping_in > 0.1f) ? 0.99999f : 1.0f;

			if (kickTrigger.process(inputs[KICK_TRIG_IN].getVoltage() + params[KICK_PARAM].getValue()))
			{
				kickPendulums();
			}

			p0.length = length_ratio;
			p1.length = 1.0f - length_ratio;
			p0.mass = p0.length * 10.f;
			p1.mass = p1.length * 10.f;
			
			if (integrationMode == IntegrationMode::RK4) {
				// initial conditions (from last frame)
				float x[4] = {p0.theta, p0.vel, p1.theta, p1.vel};
				dsp::stepRK4(0.f, dt, x, 4, [&](float time, float x[4], float dxdt[4]){
					Pendulum _p0 = p0;
					_p0.theta = x[0];
					_p0.vel = x[1];
					Pendulum _p1 = p1;
					_p1.theta = x[2];
					_p1.vel = x[3];

					dxdt[0] = _p0.vel;
					dxdt[2] = _p1.vel;
					Derivative(_p0, _p1, dxdt[1], dxdt[3]);
				});
				
				p0.theta = x[0];
				p0.vel = x[1] * damping;
				p1.theta = x[2];
				p1.vel = x[3] * damping;
			} else if (integrationMode == IntegrationMode::Euler) {
				Derivative(p0, p1, p0.acc, p1.acc);
				p0.vel += p0.acc * dt;
				p1.vel += p1.acc * dt;
				p0.vel *= damping;
				p1.vel *= damping;
				p0.theta += p0.vel * dt;
				p1.theta += p1.vel * dt;
			}

			if (p0.theta > PI*2) 
				p0.theta -= PI*2;
			if (p0.theta < PI*2)
				p0.theta += PI*2;
			if (p1.theta > PI * 2)
				p1.theta -= PI * 2;
			if (p1.theta < PI * 2)
				p1.theta += PI * 2;

			// Output pendulum 1 x position.
			p0.x = 0.0f + (p0.length * std::sin(p0.theta));
			p0.y = 0.0f - (p0.length * std::cos(p0.theta)) * -1.0f;
			p1.x = p0.x + (p1.length * std::sin(p1.theta));
			p1.y = p0.y - (p1.length * std::cos(p1.theta)) * -1.0f;
			outputs[POLY_CHAOS_OUTPUT].setVoltage(p1.x * VOLT_FIVE, 0);
			outputs[POLY_CHAOS_OUTPUT].setVoltage(p1.y * -1.f * VOLT_FIVE, 1);

			float theta1 = std::fmod(p0.theta + PI, PI*2);
			if (theta1 < 0) theta1 += PI*2;
			theta1 -= PI;

			float theta2 = std::fmod(p1.theta + PI, PI * 2);
			if (theta2 < 0)
				theta2 += PI * 2;
			theta2 -= PI;

			outputs[POLY_CHAOS_OUTPUT].setVoltage((theta1 / PI) * VOLT_FIVE, 2);
			outputs[POLY_CHAOS_OUTPUT].setVoltage((theta2 / PI) * VOLT_FIVE, 3);
			outputs[POLY_CHAOS_OUTPUT].setVoltage(p0.vel, 4);
			outputs[POLY_CHAOS_OUTPUT].setVoltage(p1.vel, 5);

			// distance from center
			float dist = std::sqrt(p1.x*p1.x + p1.y*p1.y);
			outputs[POLY_CHAOS_OUTPUT].setVoltage(dist * VOLT_MAX, 6);

			outputs[POLY_CHAOS_OUTPUT].setChannels(7);
		}
	}
};

struct PendulumWidget : OpaqueWidget {
	ChaosModule* module;

	void draw(const DrawArgs &args) override {
		OpaqueWidget::draw(args);
		if (module) {
			const float maxLen = 80.0f;
			const float center = 105.f;
			float p0_x = (module->p0.x * maxLen) + center;
			float p0_y = (module->p0.y * maxLen) + center;
			float p1_x = (module->p1.x * maxLen) + center;
			float p1_y = (module->p1.y * maxLen) + center;

			nvgBeginPath(args.vg);
			nvgStrokeWidth(args.vg, 1);
			nvgStrokeColor(args.vg, nvgRGBA(0xED, 0x1B, 0x31, 0xFF));
			nvgMoveTo(args.vg, center, center);
			nvgLineTo(args.vg, p0_x, p0_y);
			nvgCircle(args.vg, p0_x, p0_y, module->p0.mass);
			nvgClosePath(args.vg);
			nvgStroke(args.vg);

			nvgBeginPath(args.vg);
			nvgStrokeWidth(args.vg, 1);
			nvgStrokeColor(args.vg, nvgRGBA(0x7E, 0xD3, 0xEF, 0xFF));
			nvgMoveTo(args.vg, p0_x, p0_y);
			nvgLineTo(args.vg, p1_x, p1_y);
			nvgCircle(args.vg, p1_x, p1_y, module->p1.mass);
			nvgClosePath(args.vg);
			nvgStroke(args.vg);
		}
	}
};

struct ChaosWidget : ModuleWidget {
	ChaosWidget(ChaosModule* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Chaos.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		const float col01_x = 44.9;
		const float col02_x = 112.5;
		const float col03_x = 182.4;

		const float row01_y = 72.2;
		const float row02_y = 105.8;

		const float kick_x = 36.8;
		const float kick_y = 343.2;
		const float kick_btn_x = 68.7;
		const float kick_btn_y = 343.2;

		const float poly_out_x = 187.8;
		const float poly_out_y = 345.2;

		const float scope_x = 7.5;
		const float scope_y = 122.0;

		addParam(createParamCentered<RoundBlackKnob>((Vec(col01_x, row01_y)), module, ChaosModule::GRAVITY_PARAM));
		addParam(createParamCentered<CKD6>((Vec(col02_x, row01_y)), module, ChaosModule::DAMPING_PARAM));
		addParam(createParamCentered<RoundBlackKnob>((Vec(col03_x, row01_y)), module, ChaosModule::LENGTH_RATIO_PARAM));
		addParam(createParamCentered<CKD6>((Vec(kick_btn_x, kick_btn_y)), module, ChaosModule::KICK_PARAM));

		addInput(createInputCentered<PJ301MPort>((Vec(col01_x, row02_y)), module, ChaosModule::GRAVITY_IN));
		addInput(createInputCentered<PJ301MPort>((Vec(col02_x, row02_y)), module, ChaosModule::DAMPING_IN));
		addInput(createInputCentered<PJ301MPort>((Vec(col03_x, row02_y)), module, ChaosModule::RATIO_IN));
		addInput(createInputCentered<PJ301MPort>((Vec(kick_x, kick_y)), module, ChaosModule::KICK_TRIG_IN));

		addOutput(createOutputCentered<PJ301MPort>((Vec(poly_out_x, poly_out_y)), module, ChaosModule::POLY_CHAOS_OUTPUT));
		

		{
			PendulumWidget *display = new PendulumWidget();
			display->module = module;
			display->box.pos = Vec(scope_x, scope_y);
			display->box.size = Vec(210, 210);
			addChild(display);
		}
	}

	struct ChaosModuleItem : MenuItem {
		ChaosModule *module;
		void onAction(const event::Action &e) override {
			MenuItem::onAction(e);
			module->kickPendulums();
		}
	};

	struct ChaosModeItem : MenuItem {
		ChaosModule *module;
		ChaosModule::IntegrationMode mode;
		void onAction(const event::Action& e) override {
			module->integrationMode = mode;
		}
	};

	struct ChaosKickModeItem : MenuItem {
		ChaosModule *module;
		ChaosModule::KickMode mode;
		void onAction(const event::Action& e) override {
			module->kickMode = mode;
		}
	};

	void appendContextMenu(ui::Menu *menu) override {
		ChaosModule *module = dynamic_cast<ChaosModule*>(this->module);
		assert(module);

		menu->addChild(createMenuLabel("Kick"));
		menu->addChild(construct<ChaosModuleItem>(&MenuItem::text, "Kick Pendulums", &ChaosModuleItem::module, module));

		ChaosKickModeItem* keep_vel = createMenuItem<ChaosKickModeItem>("Keep Velocity on Kick");
		keep_vel->rightText = CHECKMARK(module->kickMode == ChaosModule::KickMode::KeepVelocity);
		keep_vel->module = module;
		keep_vel->mode = ChaosModule::KickMode::KeepVelocity;
		menu->addChild(keep_vel);

		ChaosKickModeItem* clear_vel = createMenuItem<ChaosKickModeItem>("Clear Velocity on Kick");
		clear_vel->rightText = CHECKMARK(module->kickMode == ChaosModule::KickMode::ClearVelocity);
		clear_vel->module = module;
		clear_vel->mode = ChaosModule::KickMode::ClearVelocity;
		menu->addChild(clear_vel);

		menu->addChild(createMenuLabel("Integrator"));

		ChaosModeItem* rk4_item = createMenuItem<ChaosModeItem>("Runge Kutta (expensive)");
		rk4_item->rightText = CHECKMARK(module->integrationMode == ChaosModule::IntegrationMode::RK4);
		rk4_item->module = module;
		rk4_item->mode = ChaosModule::IntegrationMode::RK4;
		menu->addChild(rk4_item);

		ChaosModeItem* euler_item = createMenuItem<ChaosModeItem>("Euler (cheap)");
		euler_item->rightText = CHECKMARK(module->integrationMode == ChaosModule::IntegrationMode::Euler);
		euler_item->module = module;
		euler_item->mode = ChaosModule::IntegrationMode::Euler;
		menu->addChild(euler_item);
	}
};


Model* modelChaos = createModel<ChaosModule, ChaosWidget>("Chaos");
