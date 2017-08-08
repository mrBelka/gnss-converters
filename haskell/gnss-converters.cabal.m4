name:                  gnss-converters
version:               VERSION
synopsis:              GNSS Converters.
description:           Haskell bindings for GNSS converters.
homepage:              http://github.com/swift-nav/gnss-converters
license:               BSD3
author:                Mark Fine <mark@swiftnav.com>, Joshua Gross <josh@swiftnav.com>
maintainer:            Swift Navigation <dev@swiftnav.com>
copyright:             Copyright (C) 2016, 2017 Swift Navigation, Inc.
category:              Network
build-type:            Simple
cabal-version:         >= 1.22

source-repository head
  type:                git
  location:            https://github.com/swift-nav/gnss-converters

library
  hs-source-dirs:      src
  exposed-modules:     Data.RTCM3.SBP
                     , Data.RTCM3.SBP.Observations
                     , Data.RTCM3.SBP.Time
                     , Data.RTCM3.SBP.Types
                     , SwiftNav.SBP.RTCM3
  ghc-options:         -Wall
  build-depends:       base >= 4.8 && < 5
                     , basic-prelude
                     , conduit
                     , exceptions
                     , extra
                     , lens
                     , monad-control
                     , mtl
                     , resourcet
                     , rtcm
                     , sbp
                     , time
                     , transformers-base
                     , vector
  default-language:    Haskell2010

executable sbp2rtcm3
  hs-source-dirs:      main
  main-is:             SBP2RTCM3.hs
  ghc-options:         -threaded -rtsopts -with-rtsopts=-N -Wall
  build-depends:       base
                     , basic-prelude
                     , gnss-converters
  default-language:    Haskell2010

executable rtcm32sbp
  hs-source-dirs:      main
  main-is:             RTCM32SBP.hs
  ghc-options:         -threaded -rtsopts -with-rtsopts=-N -Wall
  build-depends:       base
                     , basic-prelude
                     , binary-conduit
                     , conduit
                     , conduit-extra
                     , gnss-converters
                     , resourcet
  default-language:    Haskell2010

test-suite test
  type:                exitcode-stdio-1.0
  hs-source-dirs:      test
  main-is:             Test.hs
  other-modules:       Test.Data.RTCM3.SBP
  build-depends:       HUnit-approx
                     , base
                     , basic-prelude
                     , binary
                     , binary-conduit
                     , bytestring
                     , conduit
                     , conduit-extra
                     , gnss-converters
                     , lens
                     , resourcet
                     , rtcm
                     , sbp
                     , tasty
                     , tasty-hunit
                     , text
                     , unordered-containers
  ghc-options:         -threaded -rtsopts -with-rtsopts=-N -Wall
  default-language:    Haskell2010
