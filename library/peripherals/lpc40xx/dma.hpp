#pragma once

#include "peripherals/interrupt.hpp"
#include "platforms/targets/lpc40xx/LPC40xx.h"
#include "utility/build_info.hpp"
#include "utility/math/bit.hpp"

namespace sjsu::lpc40xx
{
class DmaController
{
 public:
  /// Terminal count interrupt enable bit.
  static constexpr auto kTerminalCountInterrupt = bit::MaskFromRange(31);
  /// Destination increment:
  /// 0 - the destination address is not incremented after each transfer.
  /// 1 - the destination address is incremented after each transfer.
  static constexpr auto kDestinationIncrement = bit::MaskFromRange(27);

  /// Source increment:
  /// 0 - the source address is not incremented after each transfer.
  /// 1 - the source address is incremented after each transfer.
  static constexpr auto kSourceIncrement = bit::MaskFromRange(26);

  /// Destination transfer width. The source and destination widths can be
  /// different from each other. The hardware automatically packs and unpacks
  /// the data as required.
  static constexpr auto kDestinationWidth = bit::MaskFromRange(21, 23);

  /// Source transfer width. The source and destination widths can be different
  /// from each other. The hardware automatically packs and unpacks the data as
  /// required.
  static constexpr auto kSourceWidth = bit::MaskFromRange(18, 20);

  /// Destination burst size. Indicates the number of transfers that make up a
  /// destination burst transfer request. This value must be set to the burst
  /// size of the destination peripheral or, if the destination is memory, to
  /// the memory boundary size. The burst size is the amount of data that is
  /// transferred when the DMACBREQ signal goes active in the destination
  /// peripheral.
  static constexpr auto kDestinationBurstSize = bit::MaskFromRange(15, 17);

  /// Source burst size. Indicates the number of transfers that make up a source
  /// burst. This value must be set to the burst size of the source peripheral,
  /// or if the source is memory, to the memory boundary size. The burst size is
  /// the amount of data that is transferred when the DMACBREQ signal goes
  /// active in the source peripheral.
  static constexpr auto kSourceBurstSize = bit::MaskFromRange(12, 14);

  /// Transfer size. This field sets the size of the transfer when the DMA
  /// controller is the flow controller, in which case the value must be set
  /// before the channel is enabled. Transfer size is updated as data transfers
  /// are completed.
  static constexpr auto kTransferSize = bit::MaskFromRange(0, 11);

  enum class TransferDirection
  {
    kMemoryToMemory                          = 0b000,
    kMemoryToPeripheral                      = 0b001,
    kPeripheralToMemory                      = 0b010,
    kSourcePeripheralToDestinationPeripheral = 0b011,
  };

  inline static LPC_GPDMA_TypeDef * reg                      = LPC_GPDMA;
  inline static std::array<LPC_GPDMACH_TypeDef *, 8> channel = {
    LPC_GPDMACH0, LPC_GPDMACH1, LPC_GPDMACH2, LPC_GPDMACH3,
    LPC_GPDMACH4, LPC_GPDMACH5, LPC_GPDMACH6, LPC_GPDMACH7,
  };

  static void Initialize()
  {
    if constexpr (build::IsPlatform(build::Platform::host))
    {
      static LPC_GPDMA_TypeDef dummy{};
      reg = &dummy;

      static std::array<LPC_GPDMACH_TypeDef, channel.size()> dummy_channels;
      channel[0] = &dummy_channels[0];
      channel[1] = &dummy_channels[1];
      channel[2] = &dummy_channels[2];
      channel[3] = &dummy_channels[3];
      channel[4] = &dummy_channels[4];
      channel[5] = &dummy_channels[5];
      channel[6] = &dummy_channels[6];
      channel[7] = &dummy_channels[7];
    }

    static constexpr auto kEnableBit = bit::MaskFromRange(0);
    static constexpr auto kBigEndian = bit::MaskFromRange(1);
    bit::Register(&reg->Config).Clear(kBigEndian).Set(kEnableBit).Save();
  }

  static void Enable(int p_channel,
                     void * source_address,
                     void * destination_address,
                     uint32_t control_settings,
                     uint32_t config_settings)
  {
    channel[p_channel]->CLLI     = 0;
    channel[p_channel]->CSrcAddr = reinterpret_cast<intptr_t>(source_address);
    channel[p_channel]->CDestAddr =
        reinterpret_cast<intptr_t>(destination_address);
    channel[p_channel]->CControl = control_settings;
    channel[p_channel]->CConfig  = config_settings;

    InterruptController::GetPlatformController().Enable({
        .interrupt_request_number = lpc40xx::DMA_IRQn,
        .interrupt_handler = [this]() { CurrentSettings().handler(*this); },
    });

    static constexpr auto kEnableBit = bit::MaskFromRange(0);
    bit::Register(&channel[p_channel]->CConfig).Set(kEnableBit).Save();
  }
};
}  // namespace sjsu::lpc40xx
