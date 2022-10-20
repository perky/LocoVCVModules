#include "plugin.hpp"
#include "dep/lodepng/lodepng.h"
#include "osdialog.h"
#include <vector>

#define IMG_WIDTH 256
#define NUM_IMG_CHANNELS 3
#define VOLT_MAX 10.f
#define POLY_CHANNELS 16

typedef unsigned int uint;
typedef unsigned char uchar;

struct TexModule : Module {
	float pixels[IMG_WIDTH * IMG_WIDTH * NUM_IMG_CHANNELS];
	uint pixelIndex[POLY_CHANNELS];
	std::string lastImagePath;
	bool bImageLoaded = false;
	dsp::BooleanTrigger autoMode;
	dsp::SchmittTrigger autoTrigger;
	uint frameIndex = 0;
	bool bAutoMode = true;
	uint channelCount = 1;

	struct PixelCoord {
		float x, y = 0;
	};
	PixelCoord pixelNormalCoords[POLY_CHANNELS];

	enum ParamIds {
		X_OFFSET,
		Y_OFFSET,
		AUTO,
		NUM_PARAMS
	};
	enum InputIds {
		X_INPUT,
		Y_INPUT,
		TRIG_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		RED_OUTPUT,
		GREEN_OUTPUT,
		BLUE_OUTPUT,
		HUE_OUTPUT,
		SATURATION_OUTPUT,
		LEVEL_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		AUTO_LIGHT,
		NUM_LIGHTS
	};

	TexModule() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(X_OFFSET, 0.f, VOLT_MAX, 0.f, "X offset", " volts");
		configParam(Y_OFFSET, 0.f, VOLT_MAX, 0.f, "Y offset", " volts");
		configSwitch(AUTO, 0.f, 1.f, 0.f, "Auto");
		configInput(X_INPUT, "X");
		configInput(Y_INPUT, "Y");
		configInput(TRIG_INPUT, "Trigger");
		configOutput(RED_OUTPUT, "Red");
		configOutput(GREEN_OUTPUT, "Green");
		configOutput(BLUE_OUTPUT, "Blue");
		configOutput(HUE_OUTPUT, "Hue");
		configOutput(SATURATION_OUTPUT, "Saturation");
		configOutput(LEVEL_OUTPUT, "Level");
	}

	json_t *dataToJson() override {
		json_t *obj = json_object();
		json_object_set_new(obj, "lastImagePath", json_string(lastImagePath.c_str()));
		json_object_set_new(obj, "autoMode", json_integer((int)bAutoMode));
		return obj;
	}

	void dataFromJson(json_t *rootJ) override {
		json_t *lastImagePathJ = json_object_get(rootJ, "lastImagePath");
		if (lastImagePathJ) {
			lastImagePath = json_string_value(lastImagePathJ);
			loadImage(lastImagePath);
		}
		json_t* autoModeJ = json_object_get(rootJ, "autoMode");
		if (autoModeJ)
			bAutoMode = json_integer_value(autoModeJ);
	}

	float pixelToVoltage(uchar pixel) {
		return ((float)pixel / 255) * VOLT_MAX;
	}

	void loadImage(std::string path) {
		std::vector<uchar> uncroppedImage;
		uint uncroppedImageWidth;
		uint uncroppedImageHeight;
		uint error = lodepng::decode(uncroppedImage, uncroppedImageWidth, uncroppedImageHeight, path, LCT_RGB);
		if (error != 0) {
			lastImagePath = "";
			bImageLoaded = false;
		} else {
			for (uint y = 0; y < IMG_WIDTH; y++) {
				for (uint x = 0; x < IMG_WIDTH; x++) {
					const uint croppedIndex = ((y * IMG_WIDTH) + x) * NUM_IMG_CHANNELS;
					if (x < uncroppedImageWidth && y < uncroppedImageHeight) {
						const uint uncroppedIndex = ((y * uncroppedImageWidth) + x) * NUM_IMG_CHANNELS;
						pixels[croppedIndex + 0] = pixelToVoltage(uncroppedImage[uncroppedIndex + 0]);
						pixels[croppedIndex + 1] = pixelToVoltage(uncroppedImage[uncroppedIndex + 1]);
						pixels[croppedIndex + 2] = pixelToVoltage(uncroppedImage[uncroppedIndex + 2]);
					} else {
						pixels[croppedIndex + 0] = 0.f;
						pixels[croppedIndex + 1] = 0.f;
						pixels[croppedIndex + 2] = 0.f;
					}
				}
			}
			lastImagePath = path;
			bImageLoaded = true;
		}
	}

	void process(const ProcessArgs& args) override {
        // Audio signals are typically +/-5V
        // https://vcvrack.com/manual/VoltageStandards.html

		if (bImageLoaded) {

			frameIndex++;

			int xInChannelCount = std::max(inputs[X_INPUT].getChannels(), 1);
			int yInChannelCount = std::max(inputs[Y_INPUT].getChannels(), 1);
			channelCount = std::max(xInChannelCount, yInChannelCount);

			if(autoMode.process(params[AUTO].getValue() > 0.f)) {
				bAutoMode = !bAutoMode;
			}

			if (bAutoMode) {
				bool bTrigger = (frameIndex % 2 == 0);
				if (inputs[TRIG_INPUT].isConnected()) {
					float trigValue = inputs[TRIG_INPUT].getVoltage();
					bTrigger = autoTrigger.process(rescale(trigValue, 0.1f, 2.f, 0.f, 1.f));
				}
				if (bTrigger) {
					const uint channel = 0;
					pixelIndex[channel] = (pixelIndex[channel] + NUM_IMG_CHANNELS) % (IMG_WIDTH*IMG_WIDTH*NUM_IMG_CHANNELS);
					const uint index1d = (pixelIndex[channel]/NUM_IMG_CHANNELS);
					pixelNormalCoords[channel].x = (index1d % IMG_WIDTH) / (float)IMG_WIDTH;
					pixelNormalCoords[channel].y = std::floor(index1d / (float)IMG_WIDTH) / (float)IMG_WIDTH;
				}
			} else {
				float xOffset = params[X_OFFSET].getValue();
				float yOffset = params[Y_OFFSET].getValue();
				for (uint channel = 0; channel < channelCount; channel++) {
					float xParam = inputs[X_INPUT].getNormalVoltage(0.f, channel);
					float yParam = inputs[Y_INPUT].getNormalVoltage(0.f, channel);
					xParam = clamp(xParam + xOffset, 0.f, VOLT_MAX) / VOLT_MAX;
					yParam = clamp(yParam + yOffset, 0.f, VOLT_MAX) / VOLT_MAX;
					pixelNormalCoords[channel].x = xParam;
					pixelNormalCoords[channel].y = yParam;
					uint xCoord = xParam * ((float)IMG_WIDTH);
					uint yCoord = yParam * ((float)IMG_WIDTH);
					pixelIndex[channel] = ((yCoord * IMG_WIDTH) + xCoord) * NUM_IMG_CHANNELS;
				}
			}
			
			for (uint channel = 0; channel < channelCount; channel++) {
				float red = pixels[pixelIndex[channel] + 0];
				float green = pixels[pixelIndex[channel] + 1];
				float blue = pixels[pixelIndex[channel] + 2];
				outputs[RED_OUTPUT].setVoltage(red, channel);
				outputs[GREEN_OUTPUT].setVoltage(green, channel);
				outputs[BLUE_OUTPUT].setVoltage(blue, channel);

				float redNorm = red / VOLT_MAX;
				float greenNorm = green / VOLT_MAX;
				float blueNorm = blue / VOLT_MAX;
				float cMax = std::max(redNorm, std::max(greenNorm, blueNorm));
				float cMin = std::min(redNorm, std::min(greenNorm, blueNorm));
				float cDelta = cMax - cMin;

				float level = (0.299 * redNorm) + (0.587 * greenNorm) + (0.114 * blueNorm);
				float saturation = 0.f;
				if (cMax != cMin) {
					if (level < 0.5f) {
						saturation = (cMax - cMin) / (cMax + cMin);
					} else {
						saturation = (cMax - cMin) / (2.0f - cMax - cMin);
					}
				}

				float hue = 0.f;
				if (cMax != cMin && saturation > 0.f) {
					if (cMax == redNorm) {
						hue = (greenNorm - blueNorm) / cDelta;
					} else if (cMax == greenNorm) {
						hue = 2.f + (blueNorm - redNorm) / cDelta;
					} else if (cMax == blueNorm) {
						hue = 4.f + (redNorm - greenNorm) / cDelta;
					}
					hue *= 60.f;
					if (hue > 0.f) {
						hue = std::floor(hue);
					} else {
						hue = std::floor(360.f - hue);
					}
				}

				outputs[HUE_OUTPUT].setVoltage((hue / 360.f) * VOLT_MAX, channel);
				outputs[SATURATION_OUTPUT].setVoltage(saturation * VOLT_MAX, channel);
				outputs[LEVEL_OUTPUT].setVoltage(level * VOLT_MAX, channel);	
			}

			for (uint outIndex = 0; outIndex < NUM_OUTPUTS; ++outIndex) {
				outputs[outIndex].setChannels(channelCount);
			}

			lights[AUTO_LIGHT].setBrightness(bAutoMode ? 1.f : 0.0f);
		}
	}
};


struct TexModuleImageDisplay : OpaqueWidget {
	TexModule* module;
	std::string imagePath = "";
	int imageWidth;
	int imageHeight;
	int imageHandle = 0;
	bool bLoadedImage = false;
	

	void drawLayer(const DrawArgs& args, int layer) override {
		if (module && layer == 1) {
			if (module->bImageLoaded && (imagePath != module->lastImagePath)) {
				imageHandle = nvgCreateImage(args.vg, module->lastImagePath.c_str(), 0);
				imagePath = module->lastImagePath;
				nvgImageSize(args.vg, imageHandle, &imageWidth, &imageHeight);
			}

			nvgBeginPath(args.vg);
			//nvgScale(args.vg, 0.89843f, 0.89843f);
			nvgScissor(args.vg, 0, 0, IMG_WIDTH, IMG_WIDTH);
		 	NVGpaint imgPaint = nvgImagePattern(args.vg, 0, 0, imageWidth, imageHeight, 0, imageHandle, 1.0f);
		 	nvgRect(args.vg, 0, 0, imageWidth, imageHeight);
		 	nvgFillPaint(args.vg, imgPaint);
		 	nvgFill(args.vg);
			nvgClosePath(args.vg);
		}
		Widget::drawLayer(args, layer);
	}
};

struct TexModuleCrosshair : OpaqueWidget {
	TexModule* module;

	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer == 1) {
			const float size = IMG_WIDTH;
			nvgBeginPath(args.vg);
			nvgStrokeWidth(args.vg, 1);
			nvgStrokeColor(args.vg, nvgRGBA(0xED, 0x1B, 0x31, 0xFF));
			for (uint channel = 0; channel < module->channelCount; channel++) {
				nvgMoveTo(args.vg, 0.f, module->pixelNormalCoords[channel].y * size);
				nvgLineTo(args.vg, size, module->pixelNormalCoords[channel].y * size);
				nvgMoveTo(args.vg, module->pixelNormalCoords[channel].x * size, 0.f);
				nvgLineTo(args.vg, module->pixelNormalCoords[channel].x * size, size);
			}
			nvgClosePath(args.vg);
			nvgStroke(args.vg);
		}
		Widget::drawLayer(args, layer);
	}
};

struct TexModuleWidget : ModuleWidget {
	TexModuleWidget(TexModule* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Tex.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		const float col01_x = 47.0;
		const float col02_x = 102.3;
		const float row01_y = 78.8;
		const float row02_y = 113.7;
		const float row03_y = 162.5;
		const float row04_y = 242.3;
		const float row05_y = 284.0;
		const float row06_y = 325.6;
		const float img_x = 151.3;
		const float img_y = 62.0;

		addInput(createInputCentered<PJ301MPort>((Vec(col01_x, row01_y)), module, TexModule::X_INPUT));
		addInput(createInputCentered<PJ301MPort>((Vec(col02_x, row01_y)), module, TexModule::Y_INPUT));

		addParam(createParamCentered<RoundBlackKnob>((Vec(col01_x, row02_y)), module, TexModule::X_OFFSET));
		addParam(createParamCentered<RoundBlackKnob>((Vec(col02_x, row02_y)), module, TexModule::Y_OFFSET));

		addParam(createParamCentered<CKD6>(Vec(col01_x, row03_y), module, TexModule::AUTO));
		addInput(createInputCentered<PJ301MPort>((Vec(col02_x, row03_y)), module, TexModule::TRIG_INPUT));
		addChild(createLight<SmallLight<GreenLight>>(Vec(col01_x + 15, row03_y - 10), module, TexModule::AUTO_LIGHT));

		addOutput(createOutputCentered<PJ301MPort>((Vec(col01_x, row04_y)), module, TexModule::HUE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>((Vec(col01_x, row05_y)), module, TexModule::SATURATION_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>((Vec(col01_x, row06_y)), module, TexModule::LEVEL_OUTPUT));

		addOutput(createOutputCentered<PJ301MPort>((Vec(col02_x, row04_y)), module, TexModule::RED_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>((Vec(col02_x, row05_y)), module, TexModule::GREEN_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>((Vec(col02_x, row06_y)), module, TexModule::BLUE_OUTPUT));

		{
			TexModuleImageDisplay *display = new TexModuleImageDisplay();
			display->module = module;
			display->box.pos = Vec(img_x, img_y);
			display->box.size = Vec(256, 256);
			addChild(display);
		}

		{
			TexModuleCrosshair *display = new TexModuleCrosshair();
			display->module = module;
			display->box.pos = Vec(img_x, img_y);
			display->box.size = Vec(256, 256);
			addChild(display);
		}
	}

	struct TexModuleItem : MenuItem {
		TexModule *module;
		void onAction(const event::Action &e) override {
			MenuItem::onAction(e);
			std::string dir = module->lastImagePath.empty() ?  asset::user("") : system::getDirectory(module->lastImagePath);
			char *path = osdialog_file(OSDIALOG_OPEN, dir.c_str(), NULL, NULL);
			if (path) {
				module->loadImage(path);
				free(path);
			}
		}
	};

	void appendContextMenu(ui::Menu *menu) override {
		TexModule *module = dynamic_cast<TexModule*>(this->module);
		assert(module);
		menu->addChild(construct<MenuLabel>());
		menu->addChild(construct<TexModuleItem>(&MenuItem::text, "Load image (png)", &TexModuleItem::module, module));
	}
};


Model* modelTex = createModel<TexModule, TexModuleWidget>("Tex");
