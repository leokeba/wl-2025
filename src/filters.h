#include <Arduino.h>
#include <vector>
#ifndef FILTERS_H
#define FILTERS_H

class SimpleExponentialFilter {
public:
    SimpleExponentialFilter(double alpha) : alpha(alpha), initialized(false), filteredValue(0.0) {}

    double filter(double value) {
        if (!initialized) {
            filteredValue = value;
            initialized = true;
        } else {
            filteredValue = alpha * value + (1 - alpha) * filteredValue;
        }
        return filteredValue;
    }
    void reset() {
        initialized = false;
        filteredValue = 0.0;
    }
    bool isInitialized() const {
        return initialized;
    }
    double getFilteredValue() const {
        return filteredValue;
    }
private:
    double alpha;
    bool initialized;
    double filteredValue;
};

class NumericalDerivativeFilter {
public:
    NumericalDerivativeFilter(double dt = 1.) : dt(dt), previousValue(0.0), initialized(false) {}

    double filter(double value) {
        if (!initialized) {
            previousValue = value;
            initialized = true;
            return 0.0; // No derivative for the first value
        }
        double derivative = (value - previousValue) / dt;
        previousValue = value;
        return derivative;
    }

    void reset() {
        initialized = false;
        previousValue = 0.0;
    }
    bool isInitialized() const {
        return initialized;
    }
    double getPreviousValue() const {
        return previousValue;
    }
    double getDt() const {
        return dt;
    }
private:
    double dt; // Time step
    double previousValue;
    bool initialized;
};

class SimpleOscillator {
public:
    SimpleOscillator(double frequency, double amplitude = 0.5, double offset = 1., double sampleRate = 10.)
        : frequency(frequency), amplitude(amplitude), offset(offset), sampleRate(sampleRate), phase(0.0) {}

    double nextSample() {
        double sample = sin(phase);
        phase += 2.0 * PI * frequency * frequencyFactor / sampleRate;
        if (phase >= 2.0 * PI) {
            phase -= 2.0 * PI; // Wrap phase to avoid overflow
        }
        return (sample + offset) * amplitude * amplitudeFactor; // Apply offset and amplitude factor
    }

    void reset() {
        phase = 0.0;
    }
    double getFrequency() const {
        return frequency;
    }
    void setFrequencyFactor(double factor) {
        frequencyFactor = factor;
        return;
    }
    void setAmplitudeFactor(double factor) {
        amplitudeFactor = factor;
    }
    double getSampleRate() const {
        return sampleRate;
    }
    void setFrequency(double newFrequency) {
        frequency = newFrequency;
    }
    void setSampleRate(double newSampleRate) {
        sampleRate = newSampleRate;
    }
    bool isInitialized() const {
        return sampleRate > 0.0;
    }
    double getPhase() const {
        return phase;
    }
    void setPhase(double newPhase) {
        phase = newPhase;
    }
    double getNextSample() {
        return nextSample();
    }
    double getCurrentSample() const {
        return sin(phase);
    }
    void setCurrentSample(double sample) {
        phase = asin(sample); // Set phase based on the current sample
        if (phase < 0) {
            phase += 2.0 * PI; // Ensure phase is non-negative
        }
    }
    double getAmplitude() const {
        return amplitude;
    }
    void setAmplitude(double amplitude) {
        amplitude = amplitude;
    }
private:
    double frequency; // Frequency in Hz
    double frequencyFactor = 1.; // Frequency factor for modulation
    double sampleRate; // Sample rate in Hz
    double phase; // Current phase of the oscillator
    double amplitude; // Current amplitude
    double amplitudeFactor = 1.; // Amplitude factor for modulation
    double offset; // Current offset
};

class MultiChannelRandomOscillator {
public:
    MultiChannelRandomOscillator(
        size_t numChannels,
        double minFreq, double maxFreq,
        double minAmp, double maxAmp,
        double sampleRate = 10.0
    ) : numChannels(numChannels),
        minFreq(minFreq), maxFreq(maxFreq),
        minAmp(minAmp), maxAmp(maxAmp),
        sampleRate(sampleRate) {
        oscillators.reserve(numChannels);
        randomize();
    }

    void randomize() {
        oscillators.clear();
        for (size_t i = 0; i < numChannels; i++) {
            double freq = random(minFreq, maxFreq);
            double amp = random(minAmp, maxAmp);
            SimpleOscillator osc(freq, amp);
            osc.setPhase(random(0.0, 2.0 * PI)); // Random initial phase
            oscillators.push_back(osc);
        }
    }

    void randomizeFrequencies() {
        for (auto& osc : oscillators) {
            double freq = random(minFreq, maxFreq);
            osc.setFrequency(freq);
            osc.setPhase(random(0.0, 2.0 * PI)); // Random initial phase
        }
    }
    void randomizeAmplitudes() {
        for (auto& osc : oscillators) {
            double amp = random(minAmp, maxAmp);
            osc.setAmplitude(amp);
            osc.setPhase(random(0.0, 2.0 * PI)); // Random initial phase
        }
    }
    void randomizePhases() {
        for (auto& osc : oscillators) {
            osc.setPhase(random(0.0, 2.0 * PI)); // Random initial phase
        }
    }

    std::vector<double> nextSample() {
        std::vector<double> samples;
        samples.reserve(numChannels);
        for (auto& osc : oscillators) {
            samples.push_back(osc.nextSample());
        }
        return samples;
    }

    void setSampleRate(double newSampleRate) {
        sampleRate = newSampleRate;
        for (auto& osc : oscillators) {
            osc.setSampleRate(sampleRate);
        }
    }

    void setFrequencyFactor(double factor) {
        for (auto& osc : oscillators) {
            osc.setFrequencyFactor(factor);
        }
    }

    void setAmplitudeFactor(double factor) {
        for (auto& osc : oscillators) {
            osc.setAmplitudeFactor(factor);
        }
    }

private:
    double random(double min, double max) {
        return min + (max - min) * (double)rand() / RAND_MAX;
    }

    size_t numChannels;
    double minFreq, maxFreq;
    double minAmp, maxAmp;
    double sampleRate;
    std::vector<SimpleOscillator> oscillators;
};

class RGBFixtureDMX {
public:
    RGBFixtureDMX(uint8_t red, uint8_t green, uint8_t blue)
        : red(red), green(green), blue(blue) {}
    
    std::vector<uint8_t> getDMXValues() const {
        return { red, green, blue };
    }
    void setRed(uint8_t value) {
        red = value;
    }
    void setGreen(uint8_t value) {
        green = value;
    }
    void setBlue(uint8_t value) {
        blue = value;
    }
    uint8_t getRed() const {
        return red;
    }
    uint8_t getGreen() const {
        return green;
    }
    uint8_t getBlue() const {
        return blue;
    }
    void setRGB(uint8_t r, uint8_t g, uint8_t b) {
        red = r;
        green = g;
        blue = b;
    }
    void setRGB(const std::vector<uint8_t>& rgb) {
        if (rgb.size() >= 3) {
            red = rgb[0];
            green = rgb[1];
            blue = rgb[2];
        }
    }
private:
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

class RGBChaser {
public:
    RGBChaser(size_t numFixtures, 
              const std::vector<double>& color1 = { 1.0, 0., 0.0 }, // Default to yellow
              const std::vector<double>& color2 = { 0.0, 0.4, 1.0 }) // Default to blue
        : color1(color1), color2(color2), numFixtures(numFixtures) {
        fixtures.reserve(numFixtures);
        for (size_t i = 0; i < numFixtures; ++i) {
            fixtures.emplace_back(0, 0, 0); // Initialize with black
        }
    }
    void setFixtureColor(size_t index, uint8_t red, uint8_t green, uint8_t blue) {
        if (index < fixtures.size()) {
            fixtures[index].setRGB(red, green, blue);
        }
    }
    void setFixtureColor(size_t index, const std::vector<uint8_t>& rgb) {
        if (index < fixtures.size()) {
            fixtures[index].setRGB(rgb);
        }
    }
    std::vector<std::vector<uint8_t>> getDMXValues() const {
        std::vector<std::vector<uint8_t>> dmxValues;
        for (const auto& fixture : fixtures) {
            dmxValues.push_back(fixture.getDMXValues());
        }
        return dmxValues;
    }
    void setAllFixturesColor(uint8_t red, uint8_t green, uint8_t blue) {
        for (auto& fixture : fixtures) {
            fixture.setRGB(red, green, blue);
        }
    }
    void setAllFixturesColor(const std::vector<uint8_t>& rgb) {
        for (auto& fixture : fixtures) {
            fixture.setRGB(rgb);
        }
    }
    size_t getNumFixtures() const {
        return fixtures.size();
    }
    RGBFixtureDMX& getFixture(size_t index) {
        if (index < fixtures.size()) {
            return fixtures[index];
        }
        throw std::out_of_range("Index out of range");
    }
    void chase() {
        // Update the phase for the chaser effect
        phase += frequency;
        if (phase >= 2.0 * PI) {
            phase -= 2.0 * PI; // Wrap phase to avoid overflow
        }

        for (size_t i = 0; i < numFixtures; ++i) {
            // Calculate position along the fixture chain [0, 1]
            double position = (double)i / numFixtures;
            
            // Scale position by sine period and add phase
            double scaledPosition = (position * sinePeriod + phase);
            
            // Create repeating pattern based on sineWidth
            double patternPosition = fmod(scaledPosition, 2.0 * PI);
            
            double fixtureSine;
            if (sineWidth >= 1.0) {
                // Normal behavior when sineWidth >= 1.0
                fixtureSine = sin(patternPosition);
            } else {
                // Compressed sine waves with gaps when sineWidth < 1.0
                double activeWidth = 2.0 * PI * sineWidth;
                if (patternPosition <= activeWidth) {
                    // Within the active sine region
                    fixtureSine = sin(patternPosition / sineWidth);
                } else {
                    // In the gap region - use lowest value
                    fixtureSine = -1.0;
                }
            }
            
            // Normalize the sine value to [0, 1]
            double normalizedSine = (fixtureSine + 1.0) / 2.0;
            // Apply sine shape factor
            double factor = normalizedSine * sineShape;
            // Interpolate between color1 and color2 based on the factor
            factor = constrain(factor, 0.0, 1.0);
            uint8_t red = constrain((color1[0] * (1 - factor) + color2[0] * factor) * 255, 0, 255);
            uint8_t green = constrain((color1[1] * (1 - factor) + color2[1] * factor) * 255, 0, 255);
            uint8_t blue = constrain((color1[2] * (1 - factor) + color2[2] * factor) * 255, 0, 255);
            fixtures[i].setRGB(red, green, blue);
        }
    }
    void setFrequency(double newFrequency) {
        frequency = newFrequency;
    }
    void setAmplitude(double newAmplitude) {
        amplitude = newAmplitude;
    }
    void setSineShape(double newSineShape) {
        sineShape = newSineShape;
    }
    void setSineWidth(double newSineWidth) {
        sineWidth = newSineWidth;
    }
    void setSinePeriod(double newSinePeriod) {
        sinePeriod = newSinePeriod;
    }
private:
    size_t numFixtures;
    double phase = 0.0; // Current phase for chaser effect
    double frequency = 0.1; // Frequency of the chaser effect
    double amplitude = 1.0; // Amplitude of the chaser effect
    double sineShape = 1.0; // Sine shape factor for the chaser effect
    double sineWidth = 1.0; // Sine width factor for the chaser effect
    double sinePeriod = 2.0 * PI; // Full sine wave period
    std::vector<double> color1;
    std::vector<double> color2;
    std::vector<RGBFixtureDMX> fixtures;
};

#endif // FILTERS_H