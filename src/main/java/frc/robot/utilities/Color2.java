/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.Objects;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Represents colors.
 * <p>Limited to 12 bits of precision.
 */
@SuppressWarnings("MemberName")
public class Color2 {
  private static final double kPrecision = Math.pow(2, -12);

  /*
   * FIRST Colors
   */

  /**
   * #1560BD.
   */
  public static final Color2 kDenim = new Color2(0.0823529412, 0.376470589, 0.7411764706);

  /**
   * #0066B3.
   */
  public static final Color2 kFirstBlue = new Color2(0.0, 0.4, 0.7019607844);

  /**
   * #ED1C24.
   */
  public static final Color2 kFirstRed = new Color2(0.9294117648, 0.1098039216, 0.1411764706);

  /*
   * Standard Colors
   */

  /**
   * #F0F8FF.
   */
  public static final Color2 kAliceBlue = new Color2(0.9411765f, 0.972549f, 1.0f);

  /**
   * #FAEBD7.
   */
  public static final Color2 kAntiqueWhite = new Color2(0.98039216f, 0.92156863f, 0.84313726f);

  /**
   * #00FFFF.
   */
  public static final Color2 kAqua = new Color2(0.0f, 1.0f, 1.0f);

  /**
   * #7FFFD4.
    */
  public static final Color2 kAquamarine = new Color2(0.49803922f, 1.0f, 0.83137256f);

  /**
   * #F0FFFF.
   */
  public static final Color2 kAzure = new Color2(0.9411765f, 1.0f, 1.0f);

  /**
   * #F5F5DC.
   */
  public static final Color2 kBeige = new Color2(0.9607843f, 0.9607843f, 0.8627451f);

  /**
   * #FFE4C4.
   */
  public static final Color2 kBisque = new Color2(1.0f, 0.89411765f, 0.76862746f);

  /**
   * #000000.
   */
  public static final Color2 kBlack = new Color2(0.0f, 0.0f, 0.0f);

  /**
   * #FFEBCD.
   */
  public static final Color2 kBlanchedAlmond = new Color2(1.0f, 0.92156863f, 0.8039216f);

  /**
   * #0000FF.
   */
  public static final Color2 kBlue = new Color2(0.0f, 0.0f, 1.0f);

  /**
   * #8A2BE2.
   */
  public static final Color2 kBlueViolet = new Color2(0.5411765f, 0.16862746f, 0.8862745f);

  /**
   * #A52A2A.
   */
  public static final Color2 kBrown = new Color2(0.64705884f, 0.16470589f, 0.16470589f);

  /**
   * #DEB887.
   */
  public static final Color2 kBurlywood = new Color2(0.87058824f, 0.72156864f, 0.5294118f);

  /**
   * #5F9EA0.
   */
  public static final Color2 kCadetBlue = new Color2(0.37254903f, 0.61960787f, 0.627451f);

  /**
   * #7FFF00.
   */
  public static final Color2 kChartreuse = new Color2(0.49803922f, 1.0f, 0.0f);

  /**
   * #D2691E.
   */
  public static final Color2 kChocolate = new Color2(0.8235294f, 0.4117647f, 0.11764706f);

  /**
   * #FF7F50.
   */
  public static final Color2 kCoral = new Color2(1.0f, 0.49803922f, 0.3137255f);

  /**
   * #6495ED.
   */
  public static final Color2 kCornflowerBlue = new Color2(0.39215687f, 0.58431375f, 0.92941177f);

  /**
   * #FFF8DC.
   */
  public static final Color2 kCornsilk = new Color2(1.0f, 0.972549f, 0.8627451f);

  /**
   * #DC143C.
   */
  public static final Color2 kCrimson = new Color2(0.8627451f, 0.078431375f, 0.23529412f);

  /**
   * #00FFFF.
   */
  public static final Color2 kCyan = new Color2(0.0f, 1.0f, 1.0f);

  /**
   * #00008B.
   */
  public static final Color2 kDarkBlue = new Color2(0.0f, 0.0f, 0.54509807f);

  /**
   * #008B8B.
   */
  public static final Color2 kDarkCyan = new Color2(0.0f, 0.54509807f, 0.54509807f);

  /**
   * #B8860B.
   */
  public static final Color2 kDarkGoldenrod = new Color2(0.72156864f, 0.5254902f, 0.043137256f);

  /**
   * #A9A9A9.
   */
  public static final Color2 kDarkGray = new Color2(0.6627451f, 0.6627451f, 0.6627451f);

  /**
   * #006400.
   */
  public static final Color2 kDarkGreen = new Color2(0.0f, 0.39215687f, 0.0f);

  /**
   * #BDB76B.
   */
  public static final Color2 kDarkKhaki = new Color2(0.7411765f, 0.7176471f, 0.41960785f);

  /**
   * #8B008B.
   */
  public static final Color2 kDarkMagenta = new Color2(0.54509807f, 0.0f, 0.54509807f);

  /**
   * #556B2F.
   */
  public static final Color2 kDarkOliveGreen = new Color2(0.33333334f, 0.41960785f, 0.18431373f);

  /**
   * #FF8C00.
   */
  public static final Color2 kDarkOrange = new Color2(1.0f, 0.54901963f, 0.0f);

  /**
   * #9932CC.
   */
  public static final Color2 kDarkOrchid = new Color2(0.6f, 0.19607843f, 0.8f);

  /**
   * #8B0000.
   */
  public static final Color2 kDarkRed = new Color2(0.54509807f, 0.0f, 0.0f);

  /**
   * #E9967A.
   */
  public static final Color2 kDarkSalmon = new Color2(0.9137255f, 0.5882353f, 0.47843137f);

  /**
   * #8FBC8F.
   */
  public static final Color2 kDarkSeaGreen = new Color2(0.56078434f, 0.7372549f, 0.56078434f);

  /**
   * #483D8B.
   */
  public static final Color2 kDarkSlateBlue = new Color2(0.28235295f, 0.23921569f, 0.54509807f);

  /**
   * #2F4F4F.
   */
  public static final Color2 kDarkSlateGray = new Color2(0.18431373f, 0.30980393f, 0.30980393f);

  /**
   * #00CED1.
   */
  public static final Color2 kDarkTurquoise = new Color2(0.0f, 0.80784315f, 0.81960785f);

  /**
   * #9400D3.
   */
  public static final Color2 kDarkViolet = new Color2(0.5803922f, 0.0f, 0.827451f);

  /**
   * #FF1493.
   */
  public static final Color2 kDeepPink = new Color2(1.0f, 0.078431375f, 0.5764706f);

  /**
   * #00BFFF.
   */
  public static final Color2 kDeepSkyBlue = new Color2(0.0f, 0.7490196f, 1.0f);

  /**
   * #696969.
   */
  public static final Color2 kDimGray = new Color2(0.4117647f, 0.4117647f, 0.4117647f);

  /**
   * #1E90FF.
   */
  public static final Color2 kDodgerBlue = new Color2(0.11764706f, 0.5647059f, 1.0f);

  /**
   * #B22222.
   */
  public static final Color2 kFirebrick = new Color2(0.69803923f, 0.13333334f, 0.13333334f);

  /**
   * #FFFAF0.
   */
  public static final Color2 kFloralWhite = new Color2(1.0f, 0.98039216f, 0.9411765f);

  /**
   * #228B22.
   */
  public static final Color2 kForestGreen = new Color2(0.13333334f, 0.54509807f, 0.13333334f);

  /**
   * #FF00FF.
   */
  public static final Color2 kFuchsia = new Color2(1.0f, 0.0f, 1.0f);

  /**
   * #DCDCDC.
   */
  public static final Color2 kGainsboro = new Color2(0.8627451f, 0.8627451f, 0.8627451f);

  /**
   * #F8F8FF.
   */
  public static final Color2 kGhostWhite = new Color2(0.972549f, 0.972549f, 1.0f);

  /**
   * #FFD700.
   */
  public static final Color2 kGold = new Color2(1.0f, 0.84313726f, 0.0f);

  /**
   * #DAA520.
   */
  public static final Color2 kGoldenrod = new Color2(0.85490197f, 0.64705884f, 0.1254902f);

  /**
   * #808080.
   */
  public static final Color2 kGray = new Color2(0.5019608f, 0.5019608f, 0.5019608f);

  /**
   * #008000.
   */
  public static final Color2 kGreen = new Color2(0.0f, 0.5019608f, 0.0f);

  /**
   * #ADFF2F.
   */
  public static final Color2 kGreenYellow = new Color2(0.6784314f, 1.0f, 0.18431373f);

  /**
   * #F0FFF0.
   */
  public static final Color2 kHoneydew = new Color2(0.9411765f, 1.0f, 0.9411765f);

  /**
   * #FF69B4.
   */
  public static final Color2 kHotPink = new Color2(1.0f, 0.4117647f, 0.7058824f);

  /**
   * #CD5C5C.
   */
  public static final Color2 kIndianRed = new Color2(0.8039216f, 0.36078432f, 0.36078432f);

  /**
   * #4B0082.
   */
  public static final Color2 kIndigo = new Color2(0.29411766f, 0.0f, 0.50980395f);

  /**
   * #FFFFF0.
   */
  public static final Color2 kIvory = new Color2(1.0f, 1.0f, 0.9411765f);

  /**
   * #F0E68C.
   */
  public static final Color2 kKhaki = new Color2(0.9411765f, 0.9019608f, 0.54901963f);

  /**
   * #E6E6FA.
   */
  public static final Color2 kLavender = new Color2(0.9019608f, 0.9019608f, 0.98039216f);

  /**
   * #FFF0F5.
   */
  public static final Color2 kLavenderBlush = new Color2(1.0f, 0.9411765f, 0.9607843f);

  /**
   * #7CFC00.
   */
  public static final Color2 kLawnGreen = new Color2(0.4862745f, 0.9882353f, 0.0f);

  /**
   * #FFFACD.
   */
  public static final Color2 kLemonChiffon = new Color2(1.0f, 0.98039216f, 0.8039216f);

  /**
   * #ADD8E6.
   */
  public static final Color2 kLightBlue = new Color2(0.6784314f, 0.84705883f, 0.9019608f);

  /**
   * #F08080.
   */
  public static final Color2 kLightCoral = new Color2(0.9411765f, 0.5019608f, 0.5019608f);

  /**
   * #E0FFFF.
   */
  public static final Color2 kLightCyan = new Color2(0.8784314f, 1.0f, 1.0f);

  /**
   * #FAFAD2.
   */
  public static final Color2 kLightGoldenrodYellow = new Color2(0.98039216f, 0.98039216f, 0.8235294f);

  /**
   * #D3D3D3.
   */
  public static final Color2 kLightGray = new Color2(0.827451f, 0.827451f, 0.827451f);

  /**
   * #90EE90.
   */
  public static final Color2 kLightGreen = new Color2(0.5647059f, 0.93333334f, 0.5647059f);

  /**
   * #FFB6C1.
   */
  public static final Color2 kLightPink = new Color2(1.0f, 0.7137255f, 0.75686276f);

  /**
   * #FFA07A.
   */
  public static final Color2 kLightSalmon = new Color2(1.0f, 0.627451f, 0.47843137f);

  /**
   * #20B2AA.
   */
  public static final Color2 kLightSeagGeen = new Color2(0.1254902f, 0.69803923f, 0.6666667f);

  /**
   * #87CEFA.
   */
  public static final Color2 kLightSkyBlue = new Color2(0.5294118f, 0.80784315f, 0.98039216f);

  /**
   * #778899.
   */
  public static final Color2 kLightSlateGray = new Color2(0.46666667f, 0.53333336f, 0.6f);

  /**
   * #B0C4DE.
   */
  public static final Color2 kLightSteellue = new Color2(0.6901961f, 0.76862746f, 0.87058824f);

  /**
   * #FFFFE0.
   */
  public static final Color2 kLightYellow = new Color2(1.0f, 1.0f, 0.8784314f);

  /**
   * #00FF00.
   */
  public static final Color2 kLime = new Color2(0.0f, 1.0f, 0.0f);

  /**
   * #32CD32.
   */
  public static final Color2 kLimeGreen = new Color2(0.19607843f, 0.8039216f, 0.19607843f);

  /**
   * #FAF0E6.
   */
  public static final Color2 kLinen = new Color2(0.98039216f, 0.9411765f, 0.9019608f);

  /**
   * #FF00FF.
   */
  public static final Color2 kMagenta = new Color2(1.0f, 0.0f, 1.0f);

  /**
   * #800000.
   */
  public static final Color2 kMaroon = new Color2(0.5019608f, 0.0f, 0.0f);

  /**
   * #66CDAA.
   */
  public static final Color2 kMediumAquamarine = new Color2(0.4f, 0.8039216f, 0.6666667f);

  /**
   * #0000CD.
   */
  public static final Color2 kMediumBlue = new Color2(0.0f, 0.0f, 0.8039216f);

  /**
   * #BA55D3.
   */
  public static final Color2 kMediumOrchid = new Color2(0.7294118f, 0.33333334f, 0.827451f);

  /**
   * #9370DB.
   */
  public static final Color2 kMediumPurple = new Color2(0.5764706f, 0.4392157f, 0.85882354f);

  /**
   * #3CB371.
   */
  public static final Color2 kMediumSeaGreen = new Color2(0.23529412f, 0.7019608f, 0.44313726f);

  /**
   * #7B68EE.
   */
  public static final Color2 kMediumSlateBlue = new Color2(0.48235294f, 0.40784314f, 0.93333334f);

  /**
   * #00FA9A.
   */
  public static final Color2 kMediumSpringGreen = new Color2(0.0f, 0.98039216f, 0.6039216f);

  /**
   * #48D1CC.
   */
  public static final Color2 kMediumTurquoise = new Color2(0.28235295f, 0.81960785f, 0.8f);

  /**
   * #C71585.
   */
  public static final Color2 kMediumVioletRed = new Color2(0.78039217f, 0.08235294f, 0.52156866f);

  /**
   * #191970.
   */
  public static final Color2 kMidnightBlue = new Color2(0.09803922f, 0.09803922f, 0.4392157f);

  /**
   * #F5FFFA.
   */
  public static final Color2 kMintcream = new Color2(0.9607843f, 1.0f, 0.98039216f);

  /**
   * #FFE4E1.
   */
  public static final Color2 kMistyRose = new Color2(1.0f, 0.89411765f, 0.88235295f);

  /**
   * #FFE4B5.
   */
  public static final Color2 kMoccasin = new Color2(1.0f, 0.89411765f, 0.70980394f);

  /**
   * #FFDEAD.
   */
  public static final Color2 kNavajoWhite = new Color2(1.0f, 0.87058824f, 0.6784314f);

  /**
   * #000080.
   */
  public static final Color2 kNavy = new Color2(0.0f, 0.0f, 0.5019608f);

  /**
   * #FDF5E6.
   */
  public static final Color2 kOldLace = new Color2(0.99215686f, 0.9607843f, 0.9019608f);

  /**
   * #808000.
   */
  public static final Color2 kOlive = new Color2(0.5019608f, 0.5019608f, 0.0f);

  /**
   * #6B8E23.
   */
  public static final Color2 kOliveDrab = new Color2(0.41960785f, 0.5568628f, 0.13725491f);

  /**
   * #FFA500.
   */
  public static final Color2 kOrange = new Color2(1.0f, 0.64705884f, 0.0f);

  /**
   * #FF4500.
   */
  public static final Color2 kOrangeRed = new Color2(1.0f, 0.27058825f, 0.0f);

  /**
   * #DA70D6.
   */
  public static final Color2 kOrchid = new Color2(0.85490197f, 0.4392157f, 0.8392157f);

  /**
   * #EEE8AA.
   */
  public static final Color2 kPaleGoldenrod = new Color2(0.93333334f, 0.9098039f, 0.6666667f);

  /**
   * #98FB98.
   */
  public static final Color2 kPaleGreen = new Color2(0.59607846f, 0.9843137f, 0.59607846f);

  /**
   * #AFEEEE.
   */
  public static final Color2 kPaleTurquoise = new Color2(0.6862745f, 0.93333334f, 0.93333334f);

  /**
   * #DB7093.
   */
  public static final Color2 kPaleVioletRed = new Color2(0.85882354f, 0.4392157f, 0.5764706f);

  /**
   * #FFEFD5.
   */
  public static final Color2 kPapayaWhip = new Color2(1.0f, 0.9372549f, 0.8352941f);

  /**
   * #FFDAB9.
   */
  public static final Color2 kPeachPuff = new Color2(1.0f, 0.85490197f, 0.7254902f);

  /**
   * #CD853F.
   */
  public static final Color2 kPeru = new Color2(0.8039216f, 0.52156866f, 0.24705882f);

  /**
   * #FFC0CB.
   */
  public static final Color2 kPink = new Color2(1.0f, 0.7529412f, 0.79607844f);

  /**
   * #DDA0DD.
   */
  public static final Color2 kPlum = new Color2(0.8666667f, 0.627451f, 0.8666667f);

  /**
   * #B0E0E6.
   */
  public static final Color2 kPowderBlue = new Color2(0.6901961f, 0.8784314f, 0.9019608f);

  /**
   * #800080.
   */
  public static final Color2 kPurple = new Color2(0.5019608f, 0.0f, 0.5019608f);

  /**
   * #FF0000.
   */
  public static final Color2 kRed = new Color2(1.0f, 0.0f, 0.0f);

  /**
   * #BC8F8F.
   */
  public static final Color2 kRosyBrown = new Color2(0.7372549f, 0.56078434f, 0.56078434f);

  /**
   * #4169E1.
   */
  public static final Color2 kRoyalBlue = new Color2(0.25490198f, 0.4117647f, 0.88235295f);

  /**
   * #8B4513.
   */
  public static final Color2 kSaddleBrown = new Color2(0.54509807f, 0.27058825f, 0.07450981f);

  /**
   * #FA8072.
   */
  public static final Color2 kSalmon = new Color2(0.98039216f, 0.5019608f, 0.44705883f);

  /**
   * #F4A460.
   */
  public static final Color2 kSandyBrown = new Color2(0.95686275f, 0.6431373f, 0.3764706f);

  /**
   * #2E8B57.
   */
  public static final Color2 kSeaGreen = new Color2(0.18039216f, 0.54509807f, 0.34117648f);

  /**
   * #FFF5EE.
   */
  public static final Color2 kSeashell = new Color2(1.0f, 0.9607843f, 0.93333334f);

  /**
   * #A0522D.
   */
  public static final Color2 kSienna = new Color2(0.627451f, 0.32156864f, 0.1764706f);

  /**
   * #C0C0C0.
   */
  public static final Color2 kSilver = new Color2(0.7529412f, 0.7529412f, 0.7529412f);

  /**
   * #87CEEB.
   */
  public static final Color2 kSkyBlue = new Color2(0.5294118f, 0.80784315f, 0.92156863f);

  /**
   * #6A5ACD.
   */
  public static final Color2 kSlateBlue = new Color2(0.41568628f, 0.3529412f, 0.8039216f);

  /**
   * #708090.
   */
  public static final Color2 kSlateGray = new Color2(0.4392157f, 0.5019608f, 0.5647059f);

  /**
   * #FFFAFA.
   */
  public static final Color2 kSnow = new Color2(1.0f, 0.98039216f, 0.98039216f);

  /**
   * #00FF7F.
   */
  public static final Color2 kSpringGreen = new Color2(0.0f, 1.0f, 0.49803922f);

  /**
   * #4682B4.
   */
  public static final Color2 kSteelBlue = new Color2(0.27450982f, 0.50980395f, 0.7058824f);

  /**
   * #D2B48C.
   */
  public static final Color2 kTan = new Color2(0.8235294f, 0.7058824f, 0.54901963f);

  /**
   * #008080.
   */
  public static final Color2 kTeal = new Color2(0.0f, 0.5019608f, 0.5019608f);

  /**
   * #D8BFD8.
   */
  public static final Color2 kThistle = new Color2(0.84705883f, 0.7490196f, 0.84705883f);

  /**
   * #FF6347.
   */
  public static final Color2 kTomato = new Color2(1.0f, 0.3882353f, 0.2784314f);

  /**
   * #40E0D0.
   */
  public static final Color2 kTurquoise = new Color2(0.2509804f, 0.8784314f, 0.8156863f);

  /**
   * #EE82EE.
   */
  public static final Color2 kViolet = new Color2(0.93333334f, 0.50980395f, 0.93333334f);

  /**
   * #F5DEB3.
   */
  public static final Color2 kWheat = new Color2(0.9607843f, 0.87058824f, 0.7019608f);

  /**
   * #FFFFFF.
   */
  public static final Color2 kWhite = new Color2(1.0f, 1.0f, 1.0f);

  /**
   * #F5F5F5.
   */
  public static final Color2 kWhiteSmoke = new Color2(0.9607843f, 0.9607843f, 0.9607843f);

  /**
   * #FFFF00.
   */
  public static final Color2 kYellow = new Color2(1.0f, 1.0f, 0.0f);

  /**
   * #9ACD32.
   */
  public static final Color2 kYellowGreen = new Color2(0.6039216f, 0.8039216f, 0.19607843f);

  public final double red;
  public final double green;
  public final double blue;

  /**
   * Constructs a Color.
   *
   * @param red   Red value (0-1)
   * @param green Green value (0-1)
   * @param blue  Blue value (0-1)
   */
  public Color2(double red, double green, double blue) {
    this.red = roundAndClamp(red);
    this.green = roundAndClamp(green);
    this.blue = roundAndClamp(blue);
  }

  /**
   * Constructs a Color from a Color8Bit.
   *
   * @param color The color
   */
  public Color2(Color8Bit color) {
    this(color.red / 255.0,
        color.green / 255.0,
        color.blue / 255.0);
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }
    if (other == null || getClass() != other.getClass()) {
      return false;
    }

    Color2 color = (Color2) other;
    return Double.compare(color.red, red) == 0
        && Double.compare(color.green, green) == 0
        && Double.compare(color.blue, blue) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(red, green, blue);
  }

  private static double roundAndClamp(double value) {
    final var rounded = Math.round(value / kPrecision) * kPrecision;
    return MathUtil.clamp(rounded, 0.0, 1.0);
  }
}
